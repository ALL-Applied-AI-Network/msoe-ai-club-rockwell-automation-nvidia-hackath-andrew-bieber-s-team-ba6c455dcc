"""Render exact CadQuery meshes and measured drawings; no synthetic imagery."""
from pathlib import Path
import argparse
import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Rectangle, Circle
import numpy as np

from build_cad import ROOT, D, make_positive_body, make_positive_pads, rail_span

COLORS = dict(body='#176ca5', pad='#edaa3c', metal='#aab6c2', ink='#172d40')


def surface(ax, shape, color, alpha=1):
    verts, triangles = shape.tessellate(.35, .15)
    verts = np.array([v.toTuple() for v in verts])
    faces = verts[np.asarray(triangles)]
    normals = np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
    light = np.array([-.4,-.6,-.7]);light/=np.linalg.norm(light)
    brightness = .65+.35*np.abs(normals@light)
    rgb = np.array(matplotlib.colors.to_rgb(color))
    mesh = Poly3DCollection(faces,facecolors=np.clip(brightness[:,None]*rgb,0,1),
                            edgecolors='none',linewidths=0,alpha=alpha)
    ax.add_collection3d(mesh)


def dim(ax,a,b,text,offset=(0,0)):
    a,b=np.asarray(a),np.asarray(b)
    ax.annotate('',xy=a,xytext=b,arrowprops=dict(arrowstyle='<->',color=COLORS['ink'],lw=1.2))
    p=(a+b)/2+offset
    ax.text(*p,text,ha='center',va='center',fontsize=10,color=COLORS['ink'],
            bbox=dict(facecolor='white',edgecolor='none',pad=1))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--reference',type=Path,default=ROOT/'reference'/'SMC-MHZL2-16D2.step')
    args=p.parse_args()
    solids=cq.importers.importStep(str(args.reference)).val().Solids()
    stock=max(solids,key=lambda s:s.Volume())
    jaws=sorted(sorted(solids,key=lambda s:s.Volume())[:2],key=lambda s:s.Center().y,reverse=True)
    body=make_positive_body();pads=make_positive_pads()
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.titleweight':'bold'})
    fig=plt.figure(figsize=(15,9),facecolor='#f4f7fa')
    for number,travel in enumerate((0,6),1):
        ax=fig.add_subplot(1,2,number,projection='3d',computed_zorder=False)
        ax.set_facecolor('#f4f7fa')
        surface(ax,stock,COLORS['metal'])
        for sign,jaw in ((1,jaws[0]),(-1,jaws[1])):
            surface(ax,jaw.translate((0,sign*(travel-6),0)),'#737f8d')
            part=body if sign>0 else body.mirror('XZ')
            surface(ax,part.translate((0,sign*travel,0)),COLORS['body'])
            for pad in pads:
                part=pad if sign>0 else pad.mirror('XZ')
                surface(ax,part.translate((0,sign*travel,0)),COLORS['pad'])
        ax.set_xlim(-28,95);ax.set_ylim(-29,29);ax.set_zlim(90,-50)
        ax.set_box_aspect((123,58,140));ax.set_proj_type('ortho')
        ax.view_init(elev=15,azim=-125)
        ax.set_axis_off()
        subtitle='CLOSED · 5.2 mm / 18.0 mm gaps' if travel==0 else 'OPEN · 17.2 mm / 30.0 mm gaps'
        ax.set_title(subtitle,pad=0,color=COLORS['ink'],fontsize=15)
    fig.suptitle('Dual-zone V2.2 · screw-retention candidate',fontsize=24,fontweight='bold',
                 x=.05,y=.97,ha='left',color=COLORS['ink'])
    fig.text(.05,.91,'Blue: two printed bodies     Amber: four 1 mm contact pads (2 narrow, 2 wide)     Grey: official SMC MHZL2-16D2 CAD',
             color='#4b6072',fontsize=12)
    fig.text(.05,.07,'Same 6 mm travel per jaw. 24 mm flat screw contact extends away from the wide zone; vial/cap geometry is unchanged.',
             color=COLORS['ink'],fontsize=11)
    fig.text(.05,.035,'Geometric fit prototype only · mounting coupon first · physical strength, TCP and grasp testing pending',
             color='#7c5123',fontsize=11)
    fig.subplots_adjust(left=.02,right=.98,top=.88,bottom=.1,wspace=-.07)
    fig.savefig(ROOT/'assembly_preview.png',dpi=150,facecolor=fig.get_facecolor())
    plt.close(fig)

    fig,(ax,bx)=plt.subplots(1,2,figsize=(15,8),gridspec_kw={'width_ratios':[1.4,1]},facecolor='white')
    primary_x0, primary_x1 = rail_span('primary')
    wide_x0, wide_x1 = rail_span('wide')
    for sign in (-1,1):
        for x,width,face in ((primary_x0,primary_x1-primary_x0,2.6),
                             (wide_x0,wide_x1-wide_x0,9)):
            if sign>0:
                ax.add_patch(Rectangle((x,face+1),width,18.45-face-1,facecolor=COLORS['body']))
                ax.add_patch(Rectangle((x,face),width,1,facecolor=COLORS['pad']))
            else:
                ax.add_patch(Rectangle((x,-18.45),width,18.45-face-1,facecolor=COLORS['body']))
                ax.add_patch(Rectangle((x,-face-1),width,1,facecolor=COLORS['pad']))
    dim(ax,(0,-2.6),(0,2.6),'5.2',(8,0))
    dim(ax,(40,-9),(40,9),'18.0',(9,0))
    dim(ax,(0,-25),(40,-25),'40.0',(0,-3))
    dim(ax,(primary_x0,-31),(primary_x1,-31),f"{D['primary_rail_length_x']:.1f}",(0,-3))
    dim(ax,(wide_x0,-36),(wide_x1,-36),f"{D['wide_rail_length_x']:.1f}",(0,-3))
    ax.text(0,22,'NARROW',ha='center',fontsize=12,weight='bold',color=COLORS['ink'])
    ax.text(40,22,'WIDE',ha='center',fontsize=12,weight='bold',color=COLORS['ink'])
    ax.text(0,27,'17.2 open',ha='center',fontsize=11,color='#4b6072')
    ax.text(40,27,'30.0 open',ha='center',fontsize=11,color='#4b6072')
    ax.set(xlim=(-28,97),ylim=(-45,34));ax.set_aspect('equal');ax.axis('off')
    ax.set_title('Contact cross-section · closed position',fontsize=16,color=COLORS['ink'],pad=20)

    bx.add_patch(Rectangle((0,0),12,15,facecolor='#dbe8f2',edgecolor=COLORS['body'],lw=2))
    for z in (4,11):
        bx.add_patch(Circle((6,z),1.8,facecolor='white',edgecolor=COLORS['ink']))
        bx.plot([3,9],[z,z],color='#798c9a',lw=.7,ls='--')
    bx.plot([6,6],[-1,16],color='#798c9a',lw=.7,ls='--')
    dim(bx,(15,4),(15,11),'7.0',(4,0))
    dim(bx,(0,-3),(12,-3),'12.0',(0,-2))
    dim(bx,(-3,0),(-3,15),'15.0',(-3,0))
    bx.annotate('Ø3.6 × 2 through\nBore axis: jaw-closing Y',xy=(6,11),xytext=(0,23),
                arrowprops=dict(arrowstyle='->',color=COLORS['ink']),fontsize=11,color=COLORS['ink'])
    bx.text(6,-10,'Coupon thickness: 6.0\nStock jaw holes: Ø3.4\nFinished contact-pad stack: 1.0',
            ha='center',va='top',fontsize=11,color=COLORS['ink'],linespacing=1.7)
    bx.set(xlim=(-10,29),ylim=(-18,29));bx.set_aspect('equal');bx.axis('off')
    bx.set_title('Mounting coupon · flat face',fontsize=16,color=COLORS['ink'],pad=20)
    fig.suptitle('Key dimensions · millimetres',fontsize=23,weight='bold',x=.05,y=.98,ha='left',color=COLORS['ink'])
    fig.text(.05,.035,'Tip: Z = 83.5 in official STEP frame. Hole centres: (X,Z) = (0,13.8), (0,20.8). Printed STLs lie flat on their backing faces.',
             color='#4b6072',fontsize=10)
    fig.subplots_adjust(top=.87,bottom=.1,wspace=.25)
    fig.savefig(ROOT/'dimensions.png',dpi=150,bbox_inches='tight')
    fig.savefig(ROOT/'dimensions.svg',bbox_inches='tight')
    plt.close(fig)


if __name__=='__main__':main()

