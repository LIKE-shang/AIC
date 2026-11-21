"""Small, read-only milestone summary. No motion permission is granted here."""
import math


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def evidence_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_value': repr(value)}
    if isinstance(value, dict):
        return {key: evidence_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [evidence_json(item) for item in value]
    return value


def reason_action(reason):
    reason = str(reason or '')
    if any(text in reason for text in ('gap_exceeded', 'processing_exceeded', 'processing_result_stale')):
        return '图像处理尚未跟上；保留自动日志，按其中最慢阶段优化。'
    if any(text in reason for text in ('no_complete_frame', 'no_target', 'target_not_unique')):
        return '未解锁并稳定支撑，使完整窗框入镜。'
    if any(text in reason for text in ('multiple', 'overlap')):
        return '让一个完整窗框成为主要目标，保持相机稳定。'
    if any(text in reason for text in ('confirming', 'confirmation_pending', 'stable_metric_geometry_required', 'unique_fresh')):
        return '保持完整窗框稳定入镜，程序自动完成连续确认。'
    if any(text in reason for text in ('depth', 'calibration', 'camera_info', 'factory')):
        return '深度测量尚未成立；具体原因已存入自动日志。'
    if 'aperture' in reason:
        return '检查框边和窗洞内的前景遮挡。'
    return '保留自动日志；当前原因见括号，无需逐项检查参数。'


def stage_summary(rows, ages, now_ros):
    """Summarize recent subscriptions; an old READY cannot pass this function."""
    live = rows.get('/window_live/status') or {}
    pose = rows.get('/window_pass_competition/status') or {}
    state = rows.get('/mavros/state') or {}
    landed = rows.get('/mavros/extended_state') or {}
    runtime = live.get('runtime') or {}
    streams = runtime.get('streams') or {}
    last = live.get('last_frame_diagnostics') or {}
    rate = pose.get('real_output_rate_hz')
    context = {'localization_hz': rate if number(rate) else None,
               'rgb_hz': (streams.get('rgb') or {}).get('callback_rate_hz'),
               'depth_hz': (streams.get('depth') or {}).get('callback_rate_hz'),
               'processed_hz': runtime.get('processed_rate_hz'),
               'target_count': live.get('target_count'),
               'confirmed_frames': live.get('confirmed_frames'),
               'version': live.get('version')}
    def result(code, title, reason, action, ready=False, preview=None):
        return dict(code=code, title=title, reason=str(reason or ''), action=action,
                    geometry_stage_ready=ready, actual_output_permitted=False,
                    small_step_completed=False, requires_flight_integration=True,
                    preview=preview, **context)
    def fresh(topic, limit):
        age = ages.get(topic)
        return number(age) and 0 <= age <= limit
    def stamped_fresh(row, limit):
        stamp = row.get('stamp')
        return number(stamp) and stamp > 0 and number(now_ros) and -.02 <= now_ros-stamp <= limit
    if (fresh('/mavros/state', 2.) and state.get('armed') is True or
            fresh('/mavros/extended_state', 2.) and landed.get('landed_state') == 2):
        return result('LAND_AND_REVIEW', '本轮地面检查结束', 'armed_or_airborne',
                      '按已验证方式人工接管降落；不用在空中等检测或聊天。')
    if not (fresh('/mavros/state', 2.) and stamped_fresh(state, 2.5) and state.get('connected') is True):
        return result('WAIT_FCU', '等待飞控连接', 'fcu_missing_or_stale', '保留定位终端，等待飞控连接完成。')
    if not (fresh('/window_pass_competition/status', 2.) and
            pose.get('real_output_state') == 'FORWARDING_CONTINUOUS_POSE' and
            number(rate) and rate >= 20. and not pose.get('real_fault') and pose.get('horizontal_flags_valid') is True):
        return result('WAIT_LOCALIZATION', '等待定位', pose.get('real_fault') or 'localization_not_ready',
                      '保留定位终端；定位未正常时先不处理飞行。')
    if not fresh('/window_live/status', 1.):
        return result('WAIT_VISION', '等待识别程序', 'vision_status_missing_or_stale', '程序启动后会自动更新；无需另开check终端。')
    for key, label in (('rgb', 'RGB'), ('depth', '深度')):
        stream = streams.get(key) or {}
        if stream.get('receipt_fresh') is not True:
            return result('WAIT_'+key.upper(), '等待'+label+'图像', key+'_stream_not_fresh',
                          '先看原始画面是否更新；启动错误已自动写入日志。')
    source_stamp = live.get('source_stamp')
    status_age = ages.get('/window_live/status')
    wall_age = runtime.get('result_age_wall_s')
    current_result = (runtime.get('result_fresh') is True and number(source_stamp) and source_stamp > 0 and
                      number(now_ros) and -.02 <= now_ros-source_stamp <= .5 and
                      number(status_age) and number(wall_age) and 0 <= wall_age+status_age <= .5)
    if not current_result:
        reason = last.get('tracking_reset_reason') or last.get('reason') or live.get('reason')
        return result('WAIT_PROCESSING', '图像已到，等待处理', reason,
                      '保持地面，程序自动记录各阶段耗时；无需反复重启。')
    if live.get('ready') is not True:
        reason = live.get('tracking_reset_reason') or live.get('reason')
        return result('WAIT_TARGET', '等待稳定窗框', reason, reason_action(reason))
    if live.get('metric_geometry_ready') is not True:
        reason = live.get('metric_reason')
        if reason in (None, '', 'live_depth_geometry_preview_only'):
            reason = live.get('metric_geometry_reason')
        return result('WAIT_METRIC', '窗框已识别，等待深度几何', reason, reason_action(reason))
    preview = live.get('metric_preview')
    evidence = (live.get('metric_geometry') or {}).get('opening_evidence') or {}
    if not isinstance(preview, dict) or evidence.get('possible_opening') is not True:
        reason = live.get('metric_reason') or 'aperture_not_established'
        return result('WAIT_APERTURE', '等待有效开口测量', reason, reason_action(reason))
    delta = preview.get('planned_lateral_step_m')
    width = preview.get('observed_opening_width_m')
    height = preview.get('observed_opening_height_m')
    if not (number(delta) and abs(delta) <= .10000001 and number(width) and width > 0 and number(height) and height > 0):
        return result('WAIT_METRIC', '测量值异常', 'invalid_preview', '保留自动日志，重新观察完整框。')
    short = dict(direction='右' if delta > .00001 else ('左' if delta < -.00001 else '居中'),
                 lateral_step_cm=round(abs(delta)*100., 1), opening_width_m=round(width, 3), opening_height_m=round(height, 3))
    return result('GEOMETRY_READY', '识别阶段通过（仅预览）', '',
                  '本入口已完成识别与几何确认；实际小步尚需飞控接口接入，当前不按A。', True, short)


def display_lines(summary):
    def hz(value):return '—' if not number(value) else '%.1f' % value
    line = '定位%sHz | RGB%s / 深度%sHz | 处理%sHz | %s' % (
        hz(summary.get('localization_hz')), hz(summary.get('rgb_hz')), hz(summary.get('depth_hz')),
        hz(summary.get('processed_hz')), summary['title'])
    preview = summary.get('preview')
    if preview:
        line += ' | 开口%.2f×%.2fm | 预览%s %.1fcm' % (
            preview['opening_width_m'], preview['opening_height_m'], preview['direction'], preview['lateral_step_cm'])
    elif summary.get('reason'):
        line += ' ('+summary['reason']+')'
    return [line, '下一步：'+summary['action']]
