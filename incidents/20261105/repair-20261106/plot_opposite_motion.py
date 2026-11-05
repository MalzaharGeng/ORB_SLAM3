"""Illustrate a recorded opposing-motion interval without changing aircraft state."""
import contextlib
import io
from pathlib import Path
import runpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT=Path(__file__).resolve().parent
with contextlib.redirect_stdout(io.StringIO()):
    d=runpy.run_path(str(ROOT/'analyze_opposite_motion.py'))
font=Path('C:/Windows/Fonts/msyh.ttc')
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams['axes.unicode_minus']=False
t=d['grid'];sel=(t>=15)&(t<=18)
fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout='constrained')
axes[0].plot(t[sel],d['rc'][sel,0],color='#2563eb',lw=2)
axes[0].set_ylabel('横滚杆量\n右正 / 左负')
axes[1].plot(t[sel],d['rollsp'][sel],color='#64748b',lw=1.8,label='目标倾角')
axes[1].plot(t[sel],d['roll'][sel],color='#2563eb',lw=2,label='估计实际倾角')
axes[1].set_ylabel('横滚角（度）\n右正 / 左负');axes[1].legend(loc='lower left',ncol=2)
axes[2].plot(t[sel],d['vb'][sel,1],color='#dc2626',lw=2)
axes[2].set_ylabel('估计水平速度（m/s）\n右正 / 左负')
for ax in axes:
    ax.axhline(0,color='#475569',lw=.8)
    ax.axvspan(16.48,17.48,color='#f59e0b',alpha=.13)
    ax.grid(alpha=.2)
axes[0].set_title('最后一次飞行：左打杆期间仍向右移动，但右移速度正在减小',loc='left',pad=14)
axes[2].set_xlabel('飞控 ULog 时间（秒）；阴影为持续左打杆时段')
fig.savefig(ROOT/'opposite_motion_explanation.png',dpi=160)
print(ROOT/'opposite_motion_explanation.png')
