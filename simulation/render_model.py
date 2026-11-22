"""Export the parameterized stock envelope and provisional sensor/head layout."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle


def main():
    root=Path(__file__).parent;cfg=json.loads((root/'config.json').read_text())
    fig,axes=plt.subplots(1,2,figsize=(11,5.2));head=cfg['geometry_boxes'][1];lo,hi=head['minimum'],head['maximum']
    ax=axes[0];ax.add_patch(Rectangle((-.35,-.155),.70,.31,facecolor='#b8c7d5',edgecolor='#304b66'))
    ax.add_patch(Rectangle((lo[0],lo[1]),hi[0]-lo[0],hi[1]-lo[1],facecolor='#edbe69',alpha=.5,edgecolor='#b77400'))
    ax.add_patch(Circle((0,0),.45,fill=False,ls='--',edgecolor='#7d8b98'))
    for point,color,label,text in [(cfg['lidar_position_body_m'],'#da6416','back lidar',(-.58,.31)),([0,0,0],'#16856b','body reference',(-.23,-.33)),(cfg['microphone_position_body_m'],'#9633a8','microphone',(.36,.39))]:
        ax.scatter(*point[:2],c=color,s=45,zorder=5);ax.annotate(label,point[:2],xytext=text,fontsize=9,arrowprops=dict(arrowstyle='-',color=color))
    ax.set(xlim=(-.7,.9),ylim=(-.6,.6),xlabel='body x (m), forward',ylabel='body y (m), left',title='Top: stock envelope 0.70 m x 0.31 m');ax.set_aspect('equal');ax.grid(alpha=.2)
    ax=axes[1];base=cfg['body_reference_height_m']
    ax.add_patch(Rectangle((-.35,0),.70,.40,facecolor='#b8c7d5',edgecolor='#304b66',alpha=.6))
    ax.add_patch(Rectangle((lo[0],base+lo[2]),hi[0]-lo[0],hi[2]-lo[2],facecolor='#edbe69',alpha=.5,edgecolor='#b77400'))
    for point,color,label,text in [(cfg['lidar_position_body_m'],'#da6416','lidar (assumed)',(-.5,.57)),([0,0,0],'#16856b','body datum',(-.26,.19)),(cfg['microphone_position_body_m'],'#9633a8','mic (assumed)',(.25,1.02))]:
        ax.scatter(point[0],base+point[2],c=color,s=45,zorder=5);ax.annotate(label,(point[0],base+point[2]),xytext=text,fontsize=9,arrowprops=dict(arrowstyle='-',color=color))
    ax.axhline(0,c='#414e5f');ax.set(xlim=(-.7,.9),ylim=(-.08,1.15),xlabel='body x (m), forward',ylabel='height above nominal ground (m)',title='Side: stock standing height 0.40 m');ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('Go2 EDU v3 | back-mounted lidar | head frame and extrinsics UNMEASURED',fontsize=12)
    fig.text(.5,.025,'Stock: unitree.com/go2 | external lidar offset [-0.15, 0, +0.15] m is a simulation assumption',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.09,1,.92]);fig.savefig(root/'validation/model_layout.png',dpi=140);plt.close(fig)

if __name__=='__main__':main()
