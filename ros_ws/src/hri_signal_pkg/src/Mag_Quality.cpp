// [KOR] Mag_Quality 클래스 구현 파일 시작.
// [ENG] Begin Mag_Quality class implementation file.
#include "hri_signal_pkg/HRI_Drone_HPP/MagQuality.hpp"

// [KOR] clamp_f 구현 시작.
// [ENG] Begin clamp_f implementation.
float Mag_Quality::clamp_f(float x, float lo, float hi) {
// [KOR] lo/hi 범위로 제한해 반환.
// [ENG] Clamp to lo/hi and return.
    return std::min(std::max(x, lo), hi);
// [KOR] clamp_f 구현 종료.
// [ENG] End clamp_f implementation.
}

// [KOR] dot 구현 시작.
// [ENG] Begin dot implementation.
float Mag_Quality::dot(const float a[3], const float b[3]) {
// [KOR] 내적 계산 결과 반환.
// [ENG] Return dot product result.
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
// [KOR] dot 구현 종료.
// [ENG] End dot implementation.
}

// [KOR] norm 구현 시작.
// [ENG] Begin norm implementation.
float Mag_Quality::norm(const float v[3]) {
// [KOR] 유클리드 노름 반환.
// [ENG] Return Euclidean norm.
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
// [KOR] norm 구현 종료.
// [ENG] End norm implementation.
}

// [KOR] normalize 구현 시작.
// [ENG] Begin normalize implementation.
bool Mag_Quality::normalize(float v[3]) {
// [KOR] 길이 계산.
// [ENG] Compute magnitude.
    float n = norm(v);
// [KOR] 너무 작으면 실패.
// [ENG] Fail if too small.
    if (n < 1e-9f) return false;
// [KOR] 단위화 수행.
// [ENG] Perform normalization.
    v[0] /= n; v[1] /= n; v[2] /= n;
// [KOR] 성공 반환.
// [ENG] Return success.
    return true;
// [KOR] normalize 구현 종료.
// [ENG] End normalize implementation.
}

// [KOR] ema 구현 시작.
// [ENG] Begin ema implementation.
float Mag_Quality::ema(float prev, float x, float alpha) const {
// [KOR] alpha를 0..1로 제한.
// [ENG] Clamp alpha to 0..1.
    alpha = clamp_f(alpha, 0.0f, 1.0f);
// [KOR] EMA 계산 반환.
// [ENG] Return EMA computation.
    return prev + alpha * (x - prev);
// [KOR] ema 구현 종료.
// [ENG] End ema implementation.
}

// [KOR] compute_UH 구현 시작(Method B).
// [ENG] Begin compute_UH implementation (Method B).
bool Mag_Quality::compute_UH(const float m[3], const float a[3], float u_h_out[3]) const {
// [KOR] 가속도에서 중력 방향 후보 g 구성.
// [ENG] Build gravity direction candidate g from accel.
    float g[3] = {a[0], a[1], a[2]};
// [KOR] g 정규화 실패 시 종료.
// [ENG] Exit if g normalization fails.
    if (!normalize(g)) return false;
// [KOR] mdotg = m·g 계산.
// [ENG] Compute mdotg = m·g.
    float mdotg = dot(m, g);
// [KOR] 수평 성분 m_h 계산.
// [ENG] Compute horizontal component m_h.
    float m_h[3] = { m[0] - mdotg*g[0], m[1] - mdotg*g[1], m[2] - mdotg*g[2] };
// [KOR] 수평 성분 노름 계산.
// [ENG] Compute horizontal norm.
    float h_norm = norm(m_h);
// [KOR] 수평 성분이 너무 작으면 실패.
// [ENG] Fail if horizontal component too small.
    if (h_norm < _Q_prams.min_h_norm) return false;
// [KOR] u_h_out[0] 저장.
// [ENG] Store u_h_out[0].
    u_h_out[0] = m_h[0] / h_norm;
// [KOR] u_h_out[1] 저장.
// [ENG] Store u_h_out[1].
    u_h_out[1] = m_h[1] / h_norm;
// [KOR] u_h_out[2] 저장.
// [ENG] Store u_h_out[2].
    u_h_out[2] = m_h[2] / h_norm;
// [KOR] 성공 반환.
// [ENG] Return success.
    return true;
// [KOR] compute_UH 구현 종료.
// [ENG] End compute_UH implementation.
}

// [KOR] freshness_score_soft 구현 시작(A안 로그).
// [ENG] Begin freshness_score_soft implementation (A-plan log).
float Mag_Quality::freshness_score_soft(uint32_t dt_age_us) const {
// [KOR] 파라미터 비정상이면 0 반환.
// [ENG] Return 0 if params invalid.
    if (_Q_prams.ttl_us == 0 || _Q_prams.ttl_us <= _Q_prams.t0_us) return 0.0f;
// [KOR] dt=0이면 1 반환.
// [ENG] If dt=0, return 1.
    if (dt_age_us == 0) return 1.0f;
// [KOR] dt>=ttl이면 0 반환.
// [ENG] If dt>=ttl, return 0.
    if (dt_age_us >= _Q_prams.ttl_us) return 0.0f;
// [KOR] floor 클램프.
// [ENG] Clamp floor.
    float floor = clamp_f(_Q_prams.ttl_floor, 0.0f, 1.0f);
// [KOR] c0 하한 적용.
// [ENG] Apply lower bound to c0.
    float c0 = std::max(_Q_prams.ttl_c0, 1e-6f);
// [KOR] c1 하한 적용.
// [ENG] Apply lower bound to c1.
    float c1 = std::max(_Q_prams.ttl_c1, 1e-6f);
// [KOR] dt<=t0 구간 분기.
// [ENG] Branch for dt<=t0 segment.
    if (dt_age_us <= _Q_prams.t0_us) {
// [KOR] x=dt/t0 계산.
// [ENG] Compute x=dt/t0.
        float x = static_cast<float>(dt_age_us) / static_cast<float>(_Q_prams.t0_us);
// [KOR] x 클램프.
// [ENG] Clamp x.
        x = clamp_f(x, 0.0f, 1.0f);
// [KOR] 정규화 로그 진행도 L 계산.
// [ENG] Compute normalized log progress L.
        float L = std::log1p(c0 * x) / std::log1p(c0);
// [KOR] S=1-(1-floor)*L 계산.
// [ENG] Compute S=1-(1-floor)*L.
        float S = 1.0f - (1.0f - floor) * L;
// [KOR] S 반환.
// [ENG] Return S.
        return clamp_f(S, 0.0f, 1.0f);
// [KOR] if 종료.
// [ENG] End if.
    }
// [KOR] y=(dt-t0)/(ttl-t0) 계산.
// [ENG] Compute y=(dt-t0)/(ttl-t0).
    float y = static_cast<float>(dt_age_us - _Q_prams.t0_us) / static_cast<float>(_Q_prams.ttl_us - _Q_prams.t0_us);
// [KOR] y 클램프.
// [ENG] Clamp y.
    y = clamp_f(y, 0.0f, 1.0f);
// [KOR] 정규화 로그 진행도 L 계산.
// [ENG] Compute normalized log progress L.
    float L = std::log1p(c1 * y) / std::log1p(c1);
// [KOR] S=floor*(1-L) 계산.
// [ENG] Compute S=floor*(1-L).
    float S = floor * (1.0f - L);
// [KOR] S 반환.
// [ENG] Return S.
    return clamp_f(S, 0.0f, 1.0f);
// [KOR] freshness_score_soft 종료.
// [ENG] End freshness_score_soft.
}

// [KOR] score_norm_short 구현 시작.
// [ENG] Begin score_norm_short implementation.
float Mag_Quality::score_norm_short(float x) const {
// [KOR] 준비 안 됐으면 1.
// [ENG] If not ready, return 1.
    if (!_short_ready || _var_short <= 0.0f) return 1.0f;
// [KOR] std 계산.
// [ENG] Compute std.
    float std_s = std::sqrt(std::max(_var_short, 1e-12f));
// [KOR] z 계산.
// [ENG] Compute z.
    float z = std::fabs(x - _mu_short) / (_Q_prams.norm_k * std_s + _Q_prams.norm_eps);
// [KOR] exp(-z^2) 반환.
// [ENG] Return exp(-z^2).
    return std::exp(-(z*z));
// [KOR] score_norm_short 종료.
// [ENG] End score_norm_short.
}

// [KOR] z_long_from 구현 시작.
// [ENG] Begin z_long_from implementation.
float Mag_Quality::z_long_from(float x, float& std_long_out) const {
// [KOR] std 출력 초기화.
// [ENG] Initialize std output.
    std_long_out = 0.0f;
// [KOR] 샘플 부족이면 z=0.
// [ENG] Return z=0 if insufficient samples.
    if (_n_long < _Q_prams.baseline_min_n || _n_long <= 1) return 0.0f;
// [KOR] 분산 계산.
// [ENG] Compute variance.
    float var = _m2_long / static_cast<float>(_n_long - 1);
// [KOR] std 계산.
// [ENG] Compute std.
    std_long_out = std::sqrt(std::max(var, 1e-12f));
// [KOR] z 계산.
// [ENG] Compute z.
    float z = std::fabs(x - _mu_long) / (_Q_prams.norm_k * std_long_out + _Q_prams.norm_eps);
// [KOR] z 반환.
// [ENG] Return z.
    return z;
// [KOR] z_long_from 종료.
// [ENG] End z_long_from.
}

// [KOR] 생성자 구현 시작.
// [ENG] Begin constructor implementation.
Mag_Quality::Mag_Quality(Mag_Quality_Params params) : _Q_prams(params) {
// [KOR] reset 호출.
// [ENG] Call reset().
    reset();
// [KOR] 생성자 종료.
// [ENG] End constructor.
}

// [KOR] reset 구현 시작.
// [ENG] Begin reset implementation.
void Mag_Quality::reset() {
// [KOR] 디버그 초기화.
// [ENG] Reset debug.
    _Debug = Mag_Quality_Debug{};
// [KOR] 타이밍 초기화.
// [ENG] Reset timing.
    _last_mag_us = 0; _last_q_us = 0;
// [KOR] prev u_h 초기화.
// [ENG] Reset prev u_h.
    _have_u_prev = false; _u_prev[0]=_u_prev[1]=_u_prev[2]=0.0f;
// [KOR] long baseline 초기화.
// [ENG] Reset long baseline.
    _long_frozen = false; _n_long = 0; _mu_long = 0.0f; _m2_long = 0.0f;
// [KOR] short baseline 초기화.
// [ENG] Reset short baseline.
    _n_short = 0; _short_ready = false; _mu_short = 0.0f; _var_short = 0.0f;
// [KOR] Q 스무딩 초기화.
// [ENG] Reset Q smoothing.
    _q_smooth = 0.0f;
// [KOR] penalty 상태 초기화.
// [ENG] Reset penalty state.
    _penalty_last = 1.0f;
// [KOR] reset 종료.
// [ENG] End reset.
}

// [KOR] Baseline_Update 구현 시작(안전 구간에서만 호출해야 함).
// [ENG] Begin Baseline_Update implementation (must be called only in safe segment).
void Mag_Quality::Baseline_Update(float x) {
// [KOR] long baseline이 고정되지 않았으면 Welford 업데이트.
// [ENG] If long baseline not frozen, update Welford stats.
    if (!_long_frozen) {
// [KOR] long 샘플 증가.
// [ENG] Increment long sample count.
        _n_long++;
// [KOR] delta 계산.
// [ENG] Compute delta.
        float delta = x - _mu_long;
// [KOR] 평균 업데이트.
// [ENG] Update mean.
        _mu_long += delta / static_cast<float>(_n_long);
// [KOR] delta2 계산.
// [ENG] Compute delta2.
        float delta2 = x - _mu_long;
// [KOR] M2 업데이트.
// [ENG] Update M2.
        _m2_long += delta * delta2;
// [KOR] 워밍업 끝나면 고정.
// [ENG] Freeze after warmup.
        if (_Q_prams.long_freeze_enable && _n_long >= _Q_prams.long_warmup_n) _long_frozen = true;
// [KOR] if 블록 종료.
// [ENG] End if block.
    }
// [KOR] short alpha 클램프.
// [ENG] Clamp short alpha.
    float a = clamp_f(_Q_prams.short_alpha, 0.0f, 1.0f);
// [KOR] 첫 short 샘플이면 초기화 후 반환.
// [ENG] If first short sample, initialize and return.
    if (_n_short == 0) {
// [KOR] 평균 초기화.
// [ENG] Initialize mean.
        _mu_short = x;
// [KOR] 분산 초기화.
// [ENG] Initialize variance.
        _var_short = 0.0f;
// [KOR] 샘플 수 1.
// [ENG] Sample count 1.
        _n_short = 1;
// [KOR] ready 갱신.
// [ENG] Update ready.
        _short_ready = (_n_short >= _Q_prams.short_min_n);
// [KOR] 반환.
// [ENG] Return.
        return;
// [KOR] if 종료.
// [ENG] End if.
    }
// [KOR] 이전 평균 저장.
// [ENG] Store previous mean.
    float mu_prev = _mu_short;
// [KOR] EWMA 평균 업데이트.
// [ENG] Update EWMA mean.
    _mu_short = mu_prev + a * (x - mu_prev);
// [KOR] 잔차 e 계산.
// [ENG] Compute residual e.
    float e = x - mu_prev;
// [KOR] EWMA 분산 업데이트(안정형).
// [ENG] Update EWMA variance (stable form).
    _var_short = (1.0f - a) * (_var_short + a * e * e);
// [KOR] 샘플 수 증가.
// [ENG] Increment sample count.
    _n_short++;
// [KOR] ready 갱신.
// [ENG] Update ready.
    if (_n_short >= _Q_prams.short_min_n) _short_ready = true;
// [KOR] Baseline_Update 종료.
// [ENG] End Baseline_Update.
}

// [KOR] Quality_Update 구현 시작.
// [ENG] Begin Quality_Update implementation.
float Mag_Quality::Quality_Update(uint32_t now_us, const float m[3], const float a[3], const float gyro[3]) {
// [KOR] 마지막 MAG 시각 저장.
// [ENG] Store last MAG timestamp.
    _last_mag_us = now_us;
// [KOR] 업데이트 이벤트는 freshness=1.
// [ENG] Update event => freshness=1.
    _Debug.S_t = 1.0f;
// [KOR] |m| 계산.
// [ENG] Compute |m|.
    float mag_norm = norm(m);
// [KOR] 디버그에 |m| 저장.
// [ENG] Store |m| in debug.
    _Debug.mag_norm = mag_norm;
// [KOR] short norm 점수 계산.
// [ENG] Compute short norm score.
    float S_n_short = score_norm_short(mag_norm);
// [KOR] 디버그 short 저장.
// [ENG] Store short score in debug.
    _Debug.S_n_short = clamp_f(S_n_short, 0.0f, 1.0f);
// [KOR] 최종 norm은 short를 기본으로 사용.
// [ENG] Use short as final norm by default.
    _Debug.S_n = _Debug.S_n_short;
// [KOR] long z-score 계산.
// [ENG] Compute long z-score.
    float std_long = 0.0f;
// [KOR] z_long 호출.
// [ENG] Call z_long.
    float z_long = z_long_from(mag_norm, std_long);
// [KOR] 디버그 z_long 저장.
// [ENG] Store z_long in debug.
    _Debug.z_long = z_long;
// [KOR] anomaly 판정 저장.
// [ENG] Store anomaly decision.
    _Debug.anom_long = (z_long > _Q_prams.z_long_th);
// [KOR] penalty 기본값.
// [ENG] Default penalty.
    float penalty = 1.0f;
// [KOR] anomaly면 penalty 설정.
// [ENG] If anomaly, set penalty.
    if (_Debug.anom_long) penalty = clamp_f(_Q_prams.anom_penalty, 0.0f, 1.0f);
// [KOR] penalty 상태 저장(틱에도 동일 적용).
// [ENG] Store penalty state (reused in onTick).
    _penalty_last = penalty;
// [KOR] ACC 품질(e_acc, S_a) 계산: 동적 가속 시 tilt-comp 오염 방지.
// [ENG] Compute ACC quality (e_acc, S_a): prevents tilt-comp poisoning under dynamic accel.
    float a_norm = norm(a);
    float e_acc = std::fabs(a_norm - 1.0f);
    _Debug.e_acc = e_acc;

    float sigma = std::max(_Q_prams.acc_sigma, 1e-6f);
    float S_a = std::exp(- (e_acc * e_acc) / (sigma * sigma));
    S_a = clamp_f(S_a, _Q_prams.acc_floor, 1.0f);
    _Debug.S_a = S_a;

// [KOR] e_acc가 큰 구간에서는 tilt-comp 기반 u_h 계산 및 u_prev 갱신을 금지.
// [ENG] If e_acc is large, forbid tilt-comp u_h computation and u_prev updates.
    bool acc_ok_for_tilt = (e_acc <= _Q_prams.acc_tilt_th);
// [KOR] u_h 버퍼 선언.
// [ENG] Declare u_h buffer.
    float u_h[3] = {0,0,0};
// [KOR] S_h 초기화.
// [ENG] Initialize S_h.
    float S_h = 0.0f;
// [KOR] u_h 계산 성공 여부.
// [ENG] u_h compute success flag.
    bool ok = false;
        if (acc_ok_for_tilt) {
        ok = compute_UH(m, a, u_h);
    }
// [KOR] 실패면 S_h=0.
// [ENG] If fail, S_h=0.
    if (!ok) {
// [KOR] 방향 정보 불신(가속 과대/수평성분 부족). u_prev는 갱신하지 않음.
// [ENG] Direction info untrusted (high accel / insufficient horizontal). Do not update u_prev.
        if (!acc_ok_for_tilt) {
            // [KOR] 가속 과대인 경우: 이전 S_h를 유지해 Q의 불필요한 펄스를 줄임.
            // [ENG] Under high accel: keep previous S_h to reduce unnecessary Q pulses.
            S_h = _Debug.S_h;
        } else {
            S_h = 0.0f;
        }
// [KOR] else 시작.
// [ENG] Else begin.
    } else {
// [KOR] 이전값/시간이 있으면 변화율 기반 S_h 계산.
// [ENG] If prev/time available, compute S_h from angular rate.
        if (_have_u_prev && _last_q_us != 0 && now_us > _last_q_us) {
// [KOR] dot 후 클램프.
// [ENG] Dot then clamp.
            float c = clamp_f(dot(u_h, _u_prev), -1.0f, 1.0f);
// [KOR] 각도 변화 계산.
// [ENG] Compute angle change.
            float d = std::acos(c);
// [KOR] dt(초) 계산.
// [ENG] Compute dt (sec).
            float dt = (now_us - _last_q_us) * 1e-6f;
// [KOR] 변화율 계산.
// [ENG] Compute rate.
            float rate = d / std::max(dt, 1e-6f);
// [KOR] gyro 노름 계산.
// [ENG] Compute gyro norm.
            float gyro_norm = norm(gyro);
// [KOR] w0 선택.
// [ENG] Select w0.
            float w0 = (gyro_norm < _Q_prams.gyro_static_th) ? _Q_prams.w0_static : _Q_prams.w0_moving;
// [KOR] 정규화 r 계산.
// [ENG] Compute normalized r.
            float r = rate / std::max(w0, 1e-6f);
// [KOR] S_h 계산.
// [ENG] Compute S_h.
            S_h = std::exp(-(r*r));
// [KOR] else(첫 샘플) 시작.
// [ENG] Else (first sample) begin.
        } else {
// [KOR] 초기값 0.8.
// [ENG] Initial 0.8.
            S_h = 0.8f;
// [KOR] 내부 else 종료.
// [ENG] End inner else.
        }
// [KOR] u_prev 갱신.
// [ENG] Update u_prev.
// [KOR] u_prev 갱신(ACC 품질이 충분할 때만).
// [ENG] Update u_prev (only when ACC quality is sufficient).
        if (acc_ok_for_tilt) {
            _u_prev[0]=u_h[0]; _u_prev[1]=u_h[1]; _u_prev[2]=u_h[2];
            _have_u_prev = true;
        }
// [KOR] 외부 else 종료.
// [ENG] End outer else.
    }
// [KOR] 디버그 S_h 저장.
// [ENG] Store S_h in debug.
    _Debug.S_h = clamp_f(S_h, 0.0f, 1.0f);

// [KOR] Q_raw 결합.
// [ENG] Combine Q_raw.
    float Q_raw = clamp_f(_Debug.S_t * _Debug.S_n * _Debug.S_h * _Debug.S_a, 0.0f, 1.0f);
// [KOR] long anomaly penalty 적용.
// [ENG] Apply long anomaly penalty.
    Q_raw = clamp_f(Q_raw * _penalty_last, 0.0f, 1.0f);
// [KOR] shaping 계산.
// [ENG] Compute shaping.
    float Q_exp = std::pow(Q_raw, _Q_prams.gamma);
// [KOR] 디버그 Q_raw 저장.
// [ENG] Store Q_raw in debug.
    _Debug.Q_raw = Q_raw;
// [KOR] 디버그 Q_exp 저장.
// [ENG] Store Q_exp in debug.
    _Debug.Q_exp = Q_exp;
// [KOR] Q 스무딩.
// [ENG] Smooth Q.
    _q_smooth = ema(_q_smooth, Q_exp, _Q_prams.q_ema_alpha);
// [KOR] 디버그 Q_smooth 저장.
// [ENG] Store Q_smooth in debug.
    _Debug.Q_smooth = clamp_f(_q_smooth, 0.0f, 1.0f);
// [KOR] 마지막 Q 시각 갱신.
// [ENG] Update last Q time.
    _last_q_us = now_us;
// [KOR] 반환.
// [ENG] Return.
    return _Debug.Q_smooth;
// [KOR] Quality_Update 종료.
// [ENG] End Quality_Update.
}

// [KOR] onTick 구현 시작.
// [ENG] Begin onTick implementation.
float Mag_Quality::onTick(uint32_t now_us) {
// [KOR] MAG 미수신 상태면 Q를 0으로 수렴.
// [ENG] If MAG never received, decay Q to 0.
    if (_last_mag_us == 0) {
// [KOR] S_t=0.
// [ENG] S_t=0.
        _Debug.S_t = 0.0f;
// [KOR] Q_raw=0.
// [ENG] Q_raw=0.
        _Debug.Q_raw = 0.0f;
// [KOR] Q_exp=0.
// [ENG] Q_exp=0.
        _Debug.Q_exp = 0.0f;
// [KOR] EMA로 0으로.
// [ENG] EMA toward 0.
        _q_smooth = ema(_q_smooth, 0.0f, _Q_prams.q_ema_alpha);
// [KOR] Q_smooth 저장.
// [ENG] Store Q_smooth.
        _Debug.Q_smooth = clamp_f(_q_smooth, 0.0f, 1.0f);
// [KOR] last_q 갱신.
// [ENG] Update last_q.
        _last_q_us = now_us;
// [KOR] 반환.
// [ENG] Return.
        return _Debug.Q_smooth;
// [KOR] if 종료.
// [ENG] End if.
    }
// [KOR] dt_age 계산.
// [ENG] Compute dt_age.
    uint32_t dt_age = (now_us >= _last_mag_us) ? (now_us - _last_mag_us) : 0;
// [KOR] freshness 계산.
// [ENG] Compute freshness.
    _Debug.S_t = freshness_score_soft(dt_age);
// [KOR] tick에서는 기존 S_n,S_h 유지하며 S_t만 반영.
// [ENG] On tick, keep S_n,S_h and apply only S_t.
    float Q_raw = clamp_f(_Debug.S_t * _Debug.S_n * _Debug.S_h * _Debug.S_a, 0.0f, 1.0f);
// [KOR] penalty도 동일 적용.
// [ENG] Apply penalty as well.
    Q_raw = clamp_f(Q_raw * _penalty_last, 0.0f, 1.0f);
// [KOR] shaping.
// [ENG] Shaping.
    float Q_exp = std::pow(Q_raw, _Q_prams.gamma);
// [KOR] 저장.
// [ENG] Store.
    _Debug.Q_raw = Q_raw;
// [KOR] 저장.
// [ENG] Store.
    _Debug.Q_exp = Q_exp;
// [KOR] EMA.
// [ENG] EMA.
    _q_smooth = ema(_q_smooth, Q_exp, _Q_prams.q_ema_alpha);
// [KOR] 저장.
// [ENG] Store.
    _Debug.Q_smooth = clamp_f(_q_smooth, 0.0f, 1.0f);
// [KOR] last_q 갱신.
// [ENG] Update last_q.
    _last_q_us = now_us;
// [KOR] 반환.
// [ENG] Return.
    return _Debug.Q_smooth;
// [KOR] onTick 종료.
// [ENG] End onTick.
}