import { esc } from './console-core.js';

export const SONIC_CONFIRMATIONS = [
  ['onsite_confirmed', '我已确认现场条件，允许启动 SONIC'],
];

export function sonicCommissioningCard() {
  return `<article class="card service-card sonic-commissioning-card" data-sonic-commissioning-card>
    <div class="service-icon"><i data-lucide="clipboard-check" aria-hidden="true"></i></div>
    <div class="row between"><h3>真机现场确认</h3><span data-sonic-commissioning-badge></span></div>
    <p>核对机器人状态与现场启动条件</p>
    <div class="service-detail" data-text="sonic-commissioning-detail">等待执行端状态</div>
    <p class="service-note">确认记录本次开机有效，最长 4 小时。</p>
    <div class="row between service-actions"><span class="tiny muted">4070 本机保存</span>
      <button class="btn sm" data-action="sonic-commissioning">检查并确认</button>
    </div>
  </article>`;
}

export function sonicCommissioningDialog() {
  return `<p>请在现场目视确认 G1 已稳定、由龙门架可靠支撑并进入调试模式。SONIC 正式启动时会释放原运动模式、进入默认站姿并初始化手部。</p>
    <div class="sonic-inspection" data-sonic-inspection role="status" aria-live="polite">正在读取机器人状态…</div>
    <div class="sonic-confirmation-list">
      ${SONIC_CONFIRMATIONS.map(([key, label]) => `<label class="check-label"><input type="checkbox" data-sonic-confirmation="${key}"><span>${esc(label)}</span></label>`).join('')}
    </div>
    <p>保存后，使用旁边的“启动控制器”按钮进入真机启动流程。</p>`;
}

export function sonicInspectionHTML(inspection) {
  return inspection.checks.map(check => `<div class="row between"><div><span>${esc(check.label)}</span>${check.detail?`<small>${esc(check.detail)}</small>`:''}</div>
    <span class="badge ${check.passed ? 'success' : 'warning'}">${check.passed ? '通过' : '待处理'}</span></div>`).join('')+
    (inspection.motion_reference?`<small>${esc(inspection.motion_reference)}</small>`:'');
}
