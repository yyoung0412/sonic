"""Run explicitly in the isolated ROS venv, after sourcing Humble."""
import json
from pathlib import Path
import time
import unittest

from console_probe_worker import PassiveProbe
from console_driver import default_config
from ros2_camera_test_support import Ros2CameraFixture

APP = Path(__file__).resolve().parents[1]


class Ros2WireTests(unittest.TestCase):
    def test_g1_zmq_complete_packet_to_ros2_and_robot_station_zmq(self):
        import cv2,json,msgpack,numpy as np,socket,subprocess,sys,tempfile,threading,zmq
        from console_support import atomic_json,read_json
        from g1_zmq_ros2_bridge import default_g1_zmq_ros2_config
        from ros2_camera_bridge import default_ros2_config

        def free_port():
            with socket.socket() as sock:
                sock.bind(("127.0.0.1",0));return sock.getsockname()[1]

        source_port, output_port = free_port(), free_port()
        source_endpoint=f"tcp://127.0.0.1:{source_port}"
        output_endpoint=f"tcp://127.0.0.1:{output_port}"
        ros=default_ros2_config();ros["domain_id"]=101
        for name,row in ros["topics"].items():
            row.update(topic=f"/_robot_station_g1_test/{name}/compressed",message_type="sensor_msgs/msg/CompressedImage")
        source=default_g1_zmq_ros2_config("sim");source.update(enabled=True,endpoint=source_endpoint)
        keys={"head":"ego_view","left_wrist":"left_wrist","right_wrist":"right_wrist"}
        config={"mode":"sim","g1_zmq_ros2":source,"ros2_camera":ros,"camera_keys":keys,
                "camera_endpoint":output_endpoint,"_worker_version":"wire-test"}
        context=zmq.Context();publisher=context.socket(zmq.PUB);publisher.setsockopt(zmq.LINGER,0);publisher.bind(source_endpoint)
        subscriber=context.socket(zmq.SUB);subscriber.setsockopt(zmq.SUBSCRIBE,b"");subscriber.setsockopt(zmq.LINGER,0);subscriber.connect(output_endpoint)
        stop=threading.Event();source_process=downstream_process=None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);config_path=root/"config.json";source_status=root/"source";downstream_status=root/"downstream"
                atomic_json(config_path,config)
                source_process=subprocess.Popen(["bash",str(APP.parent/"scripts/run_g1_zmq_ros2_bridge.sh"),"--config",str(config_path),"--directory",str(source_status)])
                downstream_process=subprocess.Popen(["bash",str(APP.parent/"scripts/run_ros2_camera_bridge.sh"),"--config",str(config_path),"--directory",str(downstream_status)])
                encoded={}
                for index,key in enumerate(keys.values()):
                    image=np.zeros((120,160,3),np.uint8);image[:,:,index]=180
                    ok,jpeg=cv2.imencode(".jpg",image);self.assertTrue(ok);encoded[key]=jpeg.tobytes()
                def publish():
                    time.sleep(.5)
                    while not stop.is_set():
                        stamp=time.time();publisher.send(msgpack.packb({"images":encoded,"timestamps":{key:stamp+index*.001 for index,key in enumerate(keys.values())}},use_bin_type=True));stop.wait(.04)
                thread=threading.Thread(target=publish,daemon=True);thread.start()
                deadline=time.monotonic()+12;packet=None
                while time.monotonic()<deadline:
                    if subscriber.poll(100):
                        candidate=msgpack.unpackb(subscriber.recv(),raw=False)
                        if set(candidate.get("images",{}))==set(keys.values()):packet=candidate;break
                    if source_process.poll() is not None or downstream_process.poll() is not None:break
                self.assertIsNotNone(packet)
                status_deadline=time.monotonic()+2
                while time.monotonic()<status_deadline and (
                    read_json(source_status/"status.json").get("state")!="streaming"
                    or read_json(downstream_status/"status.json").get("state")!="streaming"
                ):
                    time.sleep(.05)
                self.assertEqual(read_json(source_status/"status.json").get("state"),"streaming")
                self.assertEqual(read_json(downstream_status/"status.json").get("state"),"streaming")
                for index,key in enumerate(keys.values()):
                    frame=cv2.imdecode(np.frombuffer(packet["images"][key],np.uint8),cv2.IMREAD_COLOR)
                    self.assertEqual(frame.shape,(480,640,3));self.assertEqual(int(frame.mean(axis=(0,1)).argmax()),index)
                stop.set();thread.join(timeout=2)
        finally:
            stop.set()
            for process in (downstream_process,source_process):
                if process is not None and process.poll() is None:
                    process.terminate();process.wait(timeout=5)
            subscriber.close();publisher.close();context.term()

    def test_real_ros_raw_compressed_color_freeze_recovery_and_passive_preview(self):
        import cv2,numpy as np,msgpack,zmq,threading
        with Ros2CameraFixture(APP) as fixture:
            context=zmq.Context();sub=context.socket(zmq.SUB);sub.setsockopt(zmq.SUBSCRIBE,b'');sub.setsockopt(zmq.LINGER,0);sub.connect(fixture.endpoint)
            try:
                fixture.wait(lambda: fixture.status().get('state')=='streaming' and fixture.status().get('packets',0)>2)
                packet=None;deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    if sub.poll(100):
                        packet=msgpack.unpackb(sub.recv(),raw=False)
                        if len(packet['images'])==3:break
                self.assertEqual(set(packet['images']),{'ego_view','left_wrist','right_wrist'})
                for key,value in packet['images'].items():
                    frame=cv2.imdecode(np.frombuffer(value,np.uint8),cv2.IMREAD_COLOR)
                    self.assertEqual(frame.shape,(480,640,3));self.assertEqual(int(frame.mean(axis=(0,1)).argmax()),{'ego_view':0,'left_wrist':1,'right_wrist':2}[key])
                (fixture.root/'preview').mkdir()
                probe=PassiveProbe(APP,fixture.root/'preview',{**default_config('sim'),'camera_endpoint':fixture.endpoint},'sim')
                thread=threading.Thread(target=probe.run_cameras,daemon=True);thread.start()
                try:
                    fixture.wait(lambda:len(probe.data['cameras'])==3)
                    self.assertTrue(all((fixture.root/'preview'/(name+'.jpg')).exists() for name in ['head','left_wrist','right_wrist']))
                    fixture.control(frozen=['left_wrist'])
                    fixture.wait(lambda:not fixture.status()['views']['left_wrist']['live']
                                 and fixture.status()['views']['head']['live']
                                 and fixture.status()['views']['right_wrist']['live'])
                    self.assertGreater(fixture.status()['views']['left_wrist']['rejected'],0)
                    fixture.control()
                    fixture.wait(lambda:fixture.status()['state']=='streaming')
                    fixture.control(disable=True)
                    fixture.wait(lambda:not any(v['live'] for v in fixture.status()['views'].values()))
                    fixture.control()
                    fixture.wait(lambda:fixture.status()['state']=='streaming')
                finally:probe.stop.set();thread.join(timeout=3)
            finally:sub.close();context.term()

    def test_domain_100_publisher_is_invisible_to_domain_99_bridge(self):
        with Ros2CameraFixture(APP,publisher_domain=100) as fixture:
            time.sleep(1.2)
            self.assertEqual(fixture.status()['packets'],0)
            fixture.start_publisher(99)
            fixture.wait(lambda:fixture.status()['state']=='streaming')
            self.assertEqual(fixture.status()['domain_id'],99)

    def test_managed_start_port_conflict_reuse_stop_and_restart(self):
        import tempfile
        from console_driver import SystemDriver
        from console_support import ConsoleError, atomic_json
        with Ros2CameraFixture(APP) as fixture, tempfile.TemporaryDirectory() as tmp:
            driver=SystemDriver(Path(tmp)/'sonic',Path(tmp)/'runtime',APP)
            patch={'camera_source':'ros2','ros2_camera':fixture.config['ros2_camera'],'camera_endpoint':fixture.endpoint}
            atomic_json(driver.config_path,{'real':patch,'sim':patch})
            try:
                with self.assertRaises(ConsoleError):driver.start_camera_source('sim')
                fixture.bridge.terminate();fixture.bridge.wait(timeout=5)
                started=driver.start_camera_source('sim')
                again=driver.start_camera_source('real')
                self.assertTrue(again['reused']);self.assertEqual(started['pid'],again['pid'])
                driver.stop_camera_source()
                self.assertEqual(driver.camera_source_status('sim')['state'],'stopped')
                replacement=driver.start_camera_source('sim')
                self.assertNotEqual(started['pid'],replacement['pid'])
            finally:driver.stop_camera_source()


if __name__=='__main__':unittest.main()
