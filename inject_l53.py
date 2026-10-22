# -*- coding: utf-8 -*-
"""反向注入：确认 lesson53（TECS 总能量控制）的断言【真的会红】。

★ 只改 lesson53 的某一行，要求它跑出非零退出码，并且**指定的那条断言**
  出现在 `[!!]` 行里。只看到 `[!!]` 不够 —— 可能是别的断言先红了。

覆盖：
  S1  能量恒等式：d(STE)/dt 的化简式加一个常数
  S2  SEB 导数：把 −V·dV 写成 +V·dV
  S3  模型基增益：K_STE2Thr 去掉 τ
  S4  权重 cap：把 min(·,1) 去掉
  S5  AP 油门映射：去掉饱和
  S6  转弯补偿：AP 因子置零
  S7  AP 失速进入阈值：0.9 改成 0.8
  S8  flare：h_ideal 的积分方向取反

注入脚本会在 `finally` 里删除临时副本；原文件不动。
"""
import io
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, 'lesson53_tecs_theory.py')
TMP = os.path.join(HERE, '_l53_mut.py')

INJECTIONS = [
    ('S1  能量化简式加 1',
     'dE_closed = V_ * (T_ - D_) / m_',
     'dE_closed = V_ * (T_ - D_) / m_ + 1.0',
     '[!!] ★ d(STE)/dt 在 γ'),
    ('S2  SEB 导数符号写反',
     'return G * dh - V * dV',
     'return G * dh + V * dV',
     '[!!] ★ d(SEB)/dt 与'),
    ('S3  K_STE2Thr 去掉 τ',
     'K = 1.0 / (tau * K_thr2STE)',
     'K = 1.0 / K_thr2STE',
     '[!!] τ=5'),
    ('S4  权重 cap 去掉',
     'sp_c, sk_c = min(2.0 - w, 1.0), min(w, 1.0)',
     'sp_c, sk_c = 2.0 - w, w',
     '[!!] ★ cap 后'),
    ('S5  AP 油门映射去掉饱和',
     'return min(max(trim_ + e / K, THRmin), THRmax)',
     'return trim_ + e / K',
     '[!!] ★ 上端'),
    ('S6  AP 转弯补偿置零',
     'ap = math.tan(phi) ** 2',
     'ap = 0.0 * math.tan(phi) ** 2',
     '[!!] ★ 在 15°–60° 内'),
    ('S7  AP 失速进入阈值 0.9→0.8',
     'ap_entry = 0.9 * Vmin',
     'ap_entry = 0.8 * Vmin',
     '[!!] AP 进入阈值'),
    ('S8  flare 积分方向取反',
     'h_ideal_new = h_ideal + DT * hdot_ref',
     'h_ideal_new = h_ideal - DT * hdot_ref',
     '[!!] ★ h_ideal 的数值导数'),
]


def run(path):
    r = subprocess.run([sys.executable, path], cwd=HERE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return r.returncode, (r.stdout or b'').decode('utf-8', 'replace')


def main():
    if not os.path.isfile(TARGET):
        print('❌ 找不到 %s' % TARGET)
        return 1
    src = io.open(TARGET, encoding='utf-8').read()

    # 基线：未注入时必须通过
    rc0, out0 = run(TARGET)
    print('基线（未注入）：EXIT=%d  %s'
          % (rc0, 'GREEN' if rc0 == 0 else 'RED'))
    if rc0 != 0:
        print(out0[-2000:])
        print('❌ 基线不过，注入没有意义')
        return 1

    n_ok = 0
    try:
        for label, old, new, marker in INJECTIONS:
            if src.count(old) != 1:
                print('❌ %s：目标串出现 %d 次（应为 1）'
                      % (label, src.count(old)))
                continue
            io.open(TMP, 'w', encoding='utf-8', newline='\n').write(
                src.replace(old, new, 1))
            try:
                rc, out = run(TMP)
                caught = (rc != 0 and marker in out)
                print('%s %-30s EXIT=%d  %s'
                      % ('✅' if caught else '❌', label, rc,
                         '捕获到目标断言' if caught else '未捕获目标断言'))
                if not caught:
                    # 打印失败断言附近，便于定位
                    for ln in out.split('\n'):
                        if marker in ln or ln.strip().startswith('[!!]'):
                            print('      %s' % ln.strip())
                else:
                    n_ok += 1
            finally:
                if os.path.isfile(TMP):
                    os.remove(TMP)
    finally:
        if os.path.isfile(TMP):
            os.remove(TMP)

    print()
    if n_ok == len(INJECTIONS):
        print('✅ 注入测试：%d/%d 通过' % (n_ok, len(INJECTIONS)))
        return 0
    print('❌ 注入测试：%d/%d 通过' % (n_ok, len(INJECTIONS)))
    return 1


if __name__ == '__main__':
    sys.exit(main())
