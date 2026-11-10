"""Independent mathematical counterexamples for the specification, not firmware tests."""
from pathlib import Path
import hashlib
import json
import math
import random


def _record_failure(message, filename, line):
    """Best-effort structured failure artifact for the acceptance ledger."""
    try:
        import json as _json
        import time as _time
        target = Path(__file__).resolve().parent.parent / 'validation' / 'audit_failure.json'
        target.write_text(_json.dumps({
            'status': 'FAIL',
            'message': str(message),
            'file': filename,
            'line': line,
            'utc': _time.strftime('%Y-%m-%dT%H:%M:%SZ', _time.gmtime()),
        }, indent=2) + '\n', encoding='utf-8')
    except Exception:
        pass


def require(condition, message="acceptance check failed"):
    """Acceptance check that survives `python -O`.

    `assert` statements are removed when Python runs with -O, which would turn a
    failed safety acceptance check into a silent pass. This helper always raises
    SystemExit with a structured message, the failing source location and a JSON
    failure artifact.
    """
    if not condition:
        import sys as _sys
        frame = _sys._getframe(1)
        _record_failure(message, frame.f_code.co_filename, frame.f_lineno)
        raise SystemExit('SPEC_AUDIT_FAILED: %s (%s:%d)' % (
            message, frame.f_code.co_filename, frame.f_lineno))

ROOT = Path(__file__).resolve().parents[2]


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def scale(a, value):
    return [value*x for x in a]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def matvec(a, b):
    return [sum(x*y for x, y in zip(row, b)) for row in a]


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def skew(a):
    return [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]]


def matadd(a, b):
    return [add(x, y) for x, y in zip(a, b)]


def eye():
    return [[float(i == j) for j in range(3)] for i in range(3)]


def rotation(vector):
    theta = math.sqrt(sum(x*x for x in vector))
    if theta < 1e-12:
        return matadd(eye(), skew(vector))
    k = skew(scale(vector, 1/theta))
    kk = matmul(k, k)
    return [[float(i == j)+math.sin(theta)*k[i][j]+(1-math.cos(theta))*kk[i][j] for j in range(3)] for i in range(3)]


def rotation_log(r):
    angle = math.acos(max(-1.0, min(1.0, (sum(r[i][i] for i in range(3))-1)/2)))
    antisymmetric = [r[2][1]-r[1][2], r[0][2]-r[2][0], r[1][0]-r[0][1]]
    if angle < 1e-6:
        return scale(antisymmetric, .5)
    if math.pi-angle < 1e-4:
        k = max(range(3), key=lambda i: r[i][i])
        axis = [0.0]*3
        axis[k] = math.sqrt(max(0.0, (r[k][k]+1)/2))
        for i in range(3):
            if i != k:
                axis[i] = (r[k][i]+r[i][k])/(4*axis[k])
        norm = math.sqrt(sum(x*x for x in axis))
        axis = scale(axis, 1/norm)
        if sum(x*y for x, y in zip(axis, antisymmetric)) < -1e-10:
            axis = scale(axis, -1)
        return scale(axis, angle)
    return scale(antisymmetric, angle/(2*math.sin(angle)))


def evaluate(coeff, u, order=0, duration=1.0):
    for _ in range(order):
        coeff = [i*coeff[i] for i in range(1, len(coeff))]
    return sum(x*u**i for i, x in enumerate(coeff))/duration**order


def quintic(p0, v0, acc0, p1, v1, acc1, duration):
    c0, c1, c2 = p0, v0*duration, .5*acc0*duration**2
    d0, d1, d2 = p1-c0-c1-c2, v1*duration-c1-2*c2, acc1*duration**2-2*c2
    return [c0, c1, c2, 10*d0-4*d1+.5*d2, -15*d0+7*d1-d2, 6*d0-3*d1+.5*d2]


def run_reference_checks():
    results = []
    random_source = random.Random(1916)
    worst_endpoint, worst_bernstein = 0.0, 0.0
    for _ in range(100):
        values = [random_source.uniform(-3, 3) for _ in range(6)]
        duration = random_source.uniform(.2, 2)
        coeff = quintic(*values, duration)
        for endpoint in [0, 1]:
            for order in range(3):
                error = abs(evaluate(coeff, endpoint, order, duration)-values[endpoint*3+order])
                worst_endpoint = max(worst_endpoint, error)
        for order in [1,2,3]:
            derivative = coeff[:]
            for _ in range(order):
                derivative = [i*derivative[i] for i in range(1, len(derivative))]
            degree = len(derivative)-1
            bernstein = [sum(derivative[k]*math.comb(i,k)/math.comb(degree,k) for k in range(i+1)) for i in range(degree+1)]
            for sample in range(101):
                value = evaluate(derivative, sample/100)
                worst_bernstein = max(worst_bernstein, min(bernstein)-value, value-max(bernstein))
    require(worst_endpoint < 1e-9 and worst_bernstein < 1e-9)
    results.append({'case':'100 arbitrary quintic endpoint and Bernstein derivative-bound fixtures', 'max_endpoint_error':worst_endpoint, 'max_bound_violation':worst_bernstein, 'status':'PASS'})

    worst_so3 = 0.0
    axes = [[1,0,0], [0,1,0], [0,0,1], [1,2,3]]
    for axis in axes:
        axis = scale(axis, 1/math.sqrt(sum(x*x for x in axis)))
        for degrees in [0, .00001, 179.9, 180, 180.1, -179.9]:
            original = rotation(scale(axis, math.radians(degrees)))
            rebuilt = rotation(rotation_log(original))
            worst_so3 = max(worst_so3, max(abs(original[i][j]-rebuilt[i][j]) for i in range(3) for j in range(3)))
    require(worst_so3 < 5e-8)
    results.append({'case':'24 SO3 near-zero and near-pi reconstruction fixtures', 'max_matrix_error':worst_so3, 'status':'PASS'})

    r = rotation([.1, -.2, .3])
    magnetic_body = [31.0, 8.0, 42.0]
    mn = matvec(r, magnetic_body)
    rho2 = mn[0]**2+mn[1]**2
    analytic = [mn[0]*mn[2]/rho2, mn[1]*mn[2]/rho2, -1.0]
    phi0 = math.atan2(mn[1],mn[0])
    finite = []
    for axis in range(3):
        delta = [0.0]*3
        delta[axis] = 1e-6
        mplus = matvec(matmul(rotation(delta),r),magnetic_body)
        mminus = matvec(matmul(rotation(scale(delta,-1)),r),magnetic_body)
        # Innovation nominal minus retracted truth, not ordinary h derivative.
        finite.append(((phi0-math.atan2(mplus[1],mplus[0]))-(phi0-math.atan2(mminus[1],mminus[0])))/2e-6)
    mag_error = max(abs(x-y) for x,y in zip(analytic,finite))
    require(mag_error < 1e-7)
    results.append({'case':'magnetic Phi-angle H central difference', 'max_error':mag_error, 'status':'PASS'})

    lever, velocity, gyro, sg = [1.2,-.3,.7], [3,-1,.5], [.2,-.1,.3], [.01,.02,-.01]
    omega0 = [6e-5,2e-5,-3e-5]
    op = [[1e-7,2e-7,0],[0,-1e-7,3e-7],[1e-7,0,2e-7]]
    ov = [[0,2e-7,-1e-7],[3e-7,0,1e-7],[0,2e-7,0]]
    lnav = matvec(r,lever)
    wnav = matvec(r,cross(gyro,lever))
    hp = matmul(skew(lnav),op)
    hv = matadd(eye(),matmul(skew(lnav),ov))
    hphi = [sub(a,b) for a,b in zip(skew(wnav),matmul(skew(omega0),skew(lnav)))]
    bdiag = [[float(i==j)/(1+sg[i]) for j in range(3)] for i in range(3)]
    sdiag = [[float(i==j)*gyro[i]/(1+sg[i]) for j in range(3)] for i in range(3)]
    hbg = [scale(x,-1) for x in matmul(matmul(r,skew(lever)),bdiag)]
    hsg = [scale(x,-1) for x in matmul(matmul(r,skew(lever)),sdiag)]
    analytic = [[*hp[i],*hv[i],*hphi[i],*hbg[i],*([0.0]*3),*hsg[i],*([0.0]*4)] for i in range(3)]
    def predicted(delta):
        rr = matmul(rotation(delta[6:9]),r)
        v = sub(velocity,delta[3:6])
        omega = sub(sub(omega0,matvec(op,delta[:3])),matvec(ov,delta[3:6]))
        corrected_gyro = [(gyro[i]*(1+sg[i])-delta[9+i])/(1+sg[i]+delta[15+i]) for i in range(3)]
        return sub(add(v,matvec(rr,cross(corrected_gyro,lever))),cross(omega,matvec(rr,lever)))
    worst_velocity = 0.0
    for column in range(22):
        plus = [0.0]*22
        plus[column] = 1e-5
        a,b = predicted(plus),predicted(scale(plus,-1))
        for row in range(3):
            value = -(a[row]-b[row])/2e-5
            worst_velocity = max(worst_velocity,abs(value-analytic[row][column]))
    require(worst_velocity < 1e-7)
    results.append({'case':'GNSS lever velocity 3x22 error-retraction H central difference (affine Earth-rate fixture)', 'max_error':worst_velocity, 'status':'PASS'})

    # Nonzero IMU->CG lever arm: public position error must include attitude P.
    cg_position = add(velocity,lnav)
    worst_cg = 0.0
    jp = [[*eye()[i],*skew(lnav)[i]] for i in range(3)]
    for column in range(6):
        plus = [0.0]*6
        plus[column] = 1e-6
        def cg(delta):
            return add(sub(velocity,delta[:3]),matvec(matmul(rotation(delta[3:]),r),lever))
        aa,bb = cg(plus),cg(scale(plus,-1))
        for row in range(3):
            worst_cg = max(worst_cg,abs(-(aa[row]-bb[row])/2e-6-jp[row][column]))
    require(worst_cg < 1e-7)
    results.append({'case':'public CG position 3x6 error-retraction Jacobian with nonzero lever', 'max_error':worst_cg, 'status':'PASS'})

    quantiles = {1:6.6348966, 3:11.3448667, 6:16.8118938}
    for dof,x in quantiles.items():
        t=x/2
        cdf = math.erf(math.sqrt(t)) if dof==1 else (math.erf(math.sqrt(t))-2*math.sqrt(t)*math.exp(-t)/math.sqrt(math.pi) if dof==3 else 1-math.exp(-t)*(1+t+t*t/2))
        require(abs(cdf-.99) < 1e-8)
    results.append({'case':'chi-square 99% quantiles checked with independent closed-form CDFs', 'status':'PASS'})

    active,preload,buffer = 'LOW','BIT0',[f'BIT{i}' for i in range(1,16)]+['ZERO1','ZERO2']
    periods=[]
    for event in range(18):
        active=preload
        periods.append(active)
        if event<len(buffer):
            preload=buffer[event]
    require(periods==[f'BIT{i}' for i in range(16)]+['ZERO1','ZERO2'])
    require(len(buffer)*4==68)
    require(248+253*8==2272 and 2272+32==2304)
    results.append({'case':'DShot 18 complete periods/68 words symbolic event model and 2304B checkpoint layout', 'status':'PASS', 'target_waveform':'NOT_RUN'})

    inputs = [ROOT/'spec'/name for name in ['02_接口契约与公共数据规范.md','04_飞控核心模块与控制系统设计.md','05_传感器采集时间基准与GNSS独立运行.md','07_实时调度与板级硬件资源设计.md','08_产品配置插件与参数管理.md']]
    report = {'scope':'DOCUMENT_REFERENCE_MATH', 'status':'PASS', 'production_numerics_SIL_HIL':'NOT_RUN', 'checks':results, 'inputs_sha256':{str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}}
    (ROOT/'spec/validation/mathematical_reference_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return {'name':'independent_geometric_and_observation_math', 'status':'PASS', 'report':'spec/validation/mathematical_reference_report.json', 'groups':len(results)}


if __name__=='__main__':
    print(json.dumps(run_reference_checks(),ensure_ascii=False))
