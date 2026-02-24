// [KOR] 헤더 가드 시작: 중복 include 방지.
// [ENG] Header guard begin: prevents multiple inclusion.
#ifndef MAG_QUALITY_HPP
// [KOR] 헤더 가드 매크로 정의.
// [ENG] Define header guard macro.
#define MAG_QUALITY_HPP

// [KOR] 고정폭 정수 타입(uint32_t 등) 사용을 위한 헤더.
// [ENG] Header for fixed-width integer types (uint32_t, etc.).
#include <cstdint>
// [KOR] 수학 함수(sqrt/acos/exp/log1p 등) 사용을 위한 헤더.
// [ENG] Header for math functions (sqrt/acos/exp/log1p, etc.).
#include <cmath>
// [KOR] std::min/std::max 등 알고리즘 유틸 사용을 위한 헤더.
// [ENG] Header for algorithm utilities (std::min/std::max, etc.).
#include <algorithm>

// [KOR] MAG 품질 계산 파라미터 구조체 정의 시작.
// [ENG] Begin parameter struct for MAG quality computation.
struct Mag_Quality_Params {
// [KOR] TTL 만료 시간(us): dt_age>=ttl_us면 freshness=0.
// [ENG] TTL expiry (us): freshness=0 if dt_age>=ttl_us.
    uint32_t ttl_us = 150000;
// [KOR] 정상 주기 기준(us): 0~t0_us 구간은 완만한 감소.
// [ENG] Nominal period (us): gentle decay over 0..t0_us.
    uint32_t t0_us = 100000;
// [KOR] dt=t0_us에서의 freshness 목표값(예: 0.95).
// [ENG] Target freshness at dt=t0_us (e.g., 0.95).
    float ttl_floor = 0.95f;
// [KOR] 0~t0_us 구간 로그 곡선 강도(c0).
// [ENG] Log curve strength c0 for 0..t0_us segment.
    float ttl_c0 = 2.0f;
// [KOR] t0_us~ttl_us 구간 로그 곡선 강도(c1).
// [ENG] Log curve strength c1 for t0_us..ttl_us segment.
    float ttl_c1 = 6.0f;
// [KOR] ACC 품질 점수 S_a = exp(-(e_acc/sigma)^2) 계산에 쓰는 sigma (g 단위).
// [ENG] Sigma (in g) used for ACC quality score S_a = exp(-(e_acc/sigma)^2).
    float acc_sigma = 0.15f;
// [KOR] tilt-comp(u_h) 계산을 허용하는 e_acc 임계값(| ||a||-1 |, g 단위).
// [ENG] e_acc threshold (| ||a||-1 | in g) to allow tilt-comp(u_h) computation.
    float acc_tilt_th = 0.30f;
// [KOR] S_a의 하한(0이면 완전 보수적). 0..1.
// [ENG] Lower bound for S_a (0 = fully conservative). 0..1.
    float acc_floor = 0.0f;
// [KOR] Q shaping 지수: Q_exp = Q_raw^gamma.
// [ENG] Q shaping exponent: Q_exp = Q_raw^gamma.
    float gamma = 3.0f;
// [KOR] Q 스무딩 EMA 알파(0..1).
// [ENG] Q smoothing EMA alpha (0..1).
    float q_ema_alpha = 0.10f;

// [KOR] 방법 B에서 수평 성분 노름 최소값(너무 작으면 실패).
// [ENG] Minimum horizontal norm for Method B (fail if too small).
    float min_h_norm = 1e-6f;
// [KOR] 정지 상태 변화율 기준(rad/s): 엄격.
// [ENG] Static angular-rate scale (rad/s): strict.
    float w0_static = 1.0f;
// [KOR] 이동 상태 변화율 기준(rad/s): 관대.
// [ENG] Moving angular-rate scale (rad/s): loose.
    float w0_moving = 6.0f;
// [KOR] gyro 노름이 이 값보다 작으면 정지로 판단(rad/s).
// [ENG] If gyro norm below this, treat as static (rad/s).
    float gyro_static_th = 0.4f;

// [KOR] long baseline 통계 신뢰 최소 샘플 수(분산 계산 가능 조건).
// [ENG] Minimum samples to trust long baseline stats (variance valid).
    uint32_t baseline_min_n = 30;
// [KOR] z-score 분모 스케일(norm_k*std)로 엄격도 조절.
// [ENG] Strictness tuning via z-score denom scale (norm_k*std).
    float norm_k = 3.0f;
// [KOR] 분모 0 방지를 위한 epsilon.
// [ENG] Epsilon to avoid division by zero.
    float norm_eps = 1e-6f;

// [KOR] long baseline을 워밍업 후 고정할지 여부.
// [ENG] Whether to freeze long baseline after warmup.
    bool long_freeze_enable = true;
// [KOR] long baseline 워밍업 샘플 수(10Hz면 50=약 5초).
// [ENG] Long baseline warmup samples (10Hz: 50≈5s).
    uint32_t long_warmup_n = 50;

// [KOR] short baseline(EWMA) 평균 적응률(10Hz: 0.05~0.2 권장).
// [ENG] Short baseline (EWMA) mean adaptation alpha (10Hz: 0.05~0.2).
    float short_alpha = 0.10f;
// [KOR] short baseline 준비 최소 샘플 수.
// [ENG] Minimum samples for short baseline to be considered ready.
    uint32_t short_min_n = 10;

// [KOR] long anomaly 임계 z(아이언/강한 외란만 잡기 위해 크게).
// [ENG] Long anomaly z-threshold (large to catch only strong disturbances).
    float z_long_th = 6.0f;
// [KOR] long anomaly 발생 시 Q에 곱할 페널티(0.1이면 90% 감소).
// [ENG] Penalty multiplier applied to Q under long anomaly (0.1 => 90% reduction).
    float anom_penalty = 0.10f;
// [KOR] 파라미터 구조체 정의 종료.
// [ENG] End parameter struct definition.
};

// [KOR] 디버그 구조체 정의 시작(튜닝/로깅용).
// [ENG] Begin debug struct definition (for tuning/logging).
struct Mag_Quality_Debug {
// [KOR] freshness 점수(0..1).
// [ENG] Freshness score (0..1).
    float S_t = 0.0f;
// [KOR] 최종 norm 점수(기본은 short).
// [ENG] Final norm score (short-based by default).
    float S_n = 1.0f;
// [KOR] short norm 점수.
// [ENG] Short norm score.
    float S_n_short = 1.0f;
// [KOR] 수평 방향 일관성 점수.
// [ENG] Horizontal direction consistency score.
    float S_h = 0.0f;

// [KOR] 결합 원시 점수.
// [ENG] Combined raw score.
    float Q_raw = 0.0f;
// [KOR] shaping 점수.
// [ENG] Shaped score.
    float Q_exp = 0.0f;
// [KOR] 스무딩 최종 점수.
// [ENG] Smoothed final score.
    float Q_smooth = 0.0f;

// [KOR] 현재 |m|.
// [ENG] Current |m|.
    float mag_norm = 0.0f;
// [KOR] long z-score(이상 감지용).
// [ENG] Long z-score (for anomaly detection).
    float z_long = 0.0f;
// [KOR] long anomaly 플래그.
// [ENG] Long anomaly flag.
    bool anom_long = false;
// [KOR] 디버그 구조체 정의 종료.
// [ENG] End debug struct definition.
// [KOR] ACC norm 오차: e_acc = | ||a|| - 1 | (g 단위).
// [ENG] ACC norm error: e_acc = | ||a|| - 1 | (in g).
    float e_acc = 0.0f;
// [KOR] ACC 품질 점수(0..1): S_a = exp(-(e_acc/sigma)^2).
// [ENG] ACC quality score(0..1): S_a = exp(-(e_acc/sigma)^2).
    float S_a = 1.0f;
};

// [KOR] MAG 품질 클래스 선언 시작.
// [ENG] Begin MAG quality class declaration.
class Mag_Quality {
public:
// [KOR] 생성자: 파라미터를 값으로 받아 저장.
// [ENG] Constructor: stores params passed by value.
    explicit Mag_Quality(Mag_Quality_Params params = Mag_Quality_Params{});
// [KOR] 새 MAG 샘플 수신 시 Q 업데이트(메인 진입점).
// [ENG] Updates Q on new MAG sample (main entry point).
    float Quality_Update(uint32_t now_us, const float m[3], const float a[3], const float gyro[3]);
// [KOR] MAG 미수신 구간에서 freshness 기반으로 Q 감소.
// [ENG] Decays Q based on freshness during MAG-missing intervals.
    float onTick(uint32_t now_us);
// [KOR] 안전 구간에서만 호출: long(Welford)+short(EWMA) baseline 업데이트.
// [ENG] Call only in safe segment: update long(Welford)+short(EWMA) baselines.
    void Baseline_Update(float mag_norm);
// [KOR] 디버그 참조 반환.
// [ENG] Returns const reference to debug info.
    const Mag_Quality_Debug& debug() const { return _Debug; }
// [KOR] 내부 상태 초기화.
// [ENG] Resets internal state.
    void reset();

private:
// [KOR] 파라미터 저장 멤버.
// [ENG] Stored parameters.
    Mag_Quality_Params _Q_prams{};
// [KOR] 디버그 저장 멤버.
// [ENG] Stored debug.
    Mag_Quality_Debug _Debug{};

// [KOR] 마지막 MAG 수신 시각(us).
// [ENG] Last MAG receive timestamp (us).
    uint32_t _last_mag_us = 0;
// [KOR] 마지막 Q 갱신 시각(us).
// [ENG] Last Q update timestamp (us).
    uint32_t _last_q_us = 0;

// [KOR] 이전 u_h 사용 가능 여부.
// [ENG] Whether previous u_h is available.
    bool _have_u_prev = false;
// [KOR] 이전 수평 자기장 단위벡터.
// [ENG] Previous horizontal unit magnet vector.
    float _u_prev[3] = {0,0,0};

// [KOR] long baseline 고정 여부(워밍업 이후).
// [ENG] Long baseline frozen flag (after warmup).
    bool _long_frozen = false;
// [KOR] long baseline 샘플 수.
// [ENG] Long baseline sample count.
    uint32_t _n_long = 0;
// [KOR] long baseline 평균.
// [ENG] Long baseline mean.
    float _mu_long = 0.0f;
// [KOR] long baseline M2(분산 누적).
// [ENG] Long baseline M2 accumulator.
    float _m2_long = 0.0f;

// [KOR] short baseline 샘플 수.
// [ENG] Short baseline sample count.
    uint32_t _n_short = 0;
// [KOR] short baseline 준비 여부.
// [ENG] Short baseline ready flag.
    bool _short_ready = false;
// [KOR] short baseline 평균(EWMA).
// [ENG] Short baseline mean (EWMA).
    float _mu_short = 0.0f;
// [KOR] short baseline 분산(EWMA).
// [ENG] Short baseline variance (EWMA).
    float _var_short = 0.0f;

// [KOR] 최종 Q 스무딩 상태(EMA).
// [ENG] Final Q smoothing state (EMA).
    float _q_smooth = 0.0f;
// [KOR] 마지막 anomaly penalty(틱에서 동일 적용).
// [ENG] Last anomaly penalty (reused during onTick).
    float _penalty_last = 1.0f;

private:
// [KOR] clamp 유틸.
// [ENG] Clamp utility.
    static float clamp_f(float x, float lo, float hi);
// [KOR] dot 유틸.
// [ENG] Dot product utility.
    static float dot(const float a[3], const float b[3]);
// [KOR] norm 유틸.
// [ENG] Norm utility.
    static float norm(const float v[3]);
// [KOR] normalize 유틸.
// [ENG] Normalize utility.
    static bool normalize(float v[3]);

// [KOR] 방법 B 핵심: tilt 보정된 수평 u_h 계산.
// [ENG] Method B core: compute tilt-compensated horizontal u_h.
    bool compute_UH(const float m[3], const float a[3], float u_h_out[3]) const;
// [KOR] EMA 계산.
// [ENG] EMA computation.
    float ema(float prev, float x, float alpha) const;

// [KOR] A안 로그형 freshness 점수 계산.
// [ENG] Computes A-plan log-shaped freshness score.
    float freshness_score_soft(uint32_t dt_age_us) const;
// [KOR] short norm score 계산.
// [ENG] Computes short norm score.
    float score_norm_short(float x) const;
// [KOR] long z-score 계산 및 std 출력.
// [ENG] Computes long z-score and outputs std.
    float z_long_from(float x, float& std_long_out) const;
// [KOR] 클래스 선언 종료.
// [ENG] End class declaration.
};

#endif