"""Sample the saved USD static environment into a metric PLY/PCD map.

Run with Isaac Sim's Python and omni.usd.libs on PYTHONPATH/LD_LIBRARY_PATH.
The output samples authored polygon surfaces (including instance proxies),
not an RGB-D scan. Materials, textures and subdivision rendering are not baked.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom


def cross2(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def earclip(vertices):
    # Project the polygon onto its dominant plane; preserve concave outlines.
    normal = np.cross(vertices, np.roll(vertices, -1, axis=0)).sum(axis=0)
    xy = np.delete(vertices, np.argmax(np.abs(normal)), axis=1)
    sign = np.sign(cross2(xy, np.roll(xy, -1, axis=0)).sum())
    remaining = list(range(len(vertices)))
    result = []
    if sign == 0:
        return []
    while len(remaining) > 3:
        for j, b in enumerate(remaining):
            a, c = remaining[j-1], remaining[(j+1) % len(remaining)]
            if sign * cross2(xy[b]-xy[a], xy[c]-xy[b]) <= 1e-14:
                continue
            others = [k for k in remaining if k not in (a,b,c)]
            q = xy[others]
            inside = ((sign*cross2(xy[b]-xy[a], q-xy[a]) >= -1e-13)
                      & (sign*cross2(xy[c]-xy[b], q-xy[b]) >= -1e-13)
                      & (sign*cross2(xy[a]-xy[c], q-xy[c]) >= -1e-13))
            if inside.any():
                continue
            result.append((a,b,c))
            remaining.pop(j)
            break
        else:
            # Remove repeated/collinear points rather than fill across a concavity.
            removable = [j for j,b in enumerate(remaining)
                         if abs(cross2(xy[b]-xy[remaining[j-1]],
                                       xy[remaining[(j+1)%len(remaining)]]-xy[b])) < 1e-12]
            if not removable:
                raise ValueError('Cannot triangulate polygon without changing its outline')
            remaining.pop(removable[0])
    result.append(tuple(remaining))
    return result


def geometry(prim, cache, unit):
    if prim.IsA(UsdGeom.Cube):
        h = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
        vertices = np.array([[-h,-h,-h],[h,-h,-h],[h,h,-h],[-h,h,-h],
                             [-h,-h,h],[h,-h,h],[h,h,h],[-h,h,h]])
        counts = np.full(6, 4)
        indices = np.array([0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7])
        holes = set()
    else:
        mesh = UsdGeom.Mesh(prim)
        vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
        holes = set(mesh.GetHoleIndicesAttr().Get() or [])
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim))
    vertices = (vertices @ matrix[:3,:3] + matrix[3,:3]) * unit
    starts = np.r_[0, np.cumsum(counts[:-1])]
    triangles = []
    for size in (3,4):
        faceids = np.flatnonzero(counts == size)
        if holes:
            faceids = np.array([i for i in faceids if i not in holes], dtype=int)
        faces = indices[starts[faceids,None] + np.arange(size)]
        if size == 3:
            triangles.append(faces)
        elif len(faces):
            v = vertices[faces]
            normals = np.cross(v, np.roll(v,-1,axis=1)).sum(axis=1)
            corner = np.cross(v-np.roll(v,1,axis=1), np.roll(v,-1,axis=1)-v)
            turn = np.einsum('nkj,nj->nk', corner, normals)
            # A concave corner must be an endpoint of the chosen diagonal.
            diag13 = (turn[:,1] < -1e-12) | (turn[:,3] < -1e-12)
            a, b = faces[~diag13], faces[diag13]
            triangles.extend([a[:,[0,1,2]], a[:,[0,2,3]],
                              b[:,[1,2,3]], b[:,[1,3,0]]])
    for i in np.flatnonzero(counts > 4):
        if i in holes:
            continue
        face = indices[starts[i]:starts[i]+counts[i]]
        local = earclip(vertices[face])
        if local:
            triangles.append(face[np.asarray(local)])
    if not triangles:
        return np.empty((0,3,3)), np.empty(0)
    tri = vertices[np.concatenate(triangles)]
    area = np.linalg.norm(np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]),axis=1)/2
    valid = np.isfinite(area) & (area > 1e-14) & np.isfinite(tri).all(axis=(1,2))
    return tri[valid], area[valid]


def write_clouds(directory, xyz, rgb):
    plydata = np.empty(len(xyz), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
                                       ('red','u1'),('green','u1'),('blue','u1')])
    for i,k in enumerate(('x','y','z')): plydata[k] = xyz[:,i]
    for i,k in enumerate(('red','green','blue')): plydata[k] = rgb[:,i]
    with (directory/'scene_map.ply').open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\ncomment units meters; frame USD_world; Z up\n'
                 'comment colors encode height, not RGB texture\n'
                 f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode())
        plydata.tofile(f)
    pcd = np.empty((len(xyz),4), dtype='<f4')
    pcd[:,:3] = xyz
    pcd[:,3].view('<u4')[:] = (rgb[:,0].astype(np.uint32)<<16) | (rgb[:,1].astype(np.uint32)<<8) | rgb[:,2]
    with (directory/'scene_map.pcd').open('wb') as f:
        f.write((f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n'
                 f'COUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(xyz)}\nDATA binary\n').encode())
        pcd.tofile(f)


def main():
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parent
    parser.add_argument('--scene',type=Path,default=root.parent/'runtime/localization_scene.local.usda')
    parser.add_argument('--output',type=Path,default=root.parent/'data/exported-map')
    parser.add_argument('--voxel',type=float,default=.02)
    args=parser.parse_args()
    assert args.voxel > 0
    args.output.mkdir(exist_ok=True,parents=True)
    stage=Usd.Stage.Open(str(args.scene))
    unit=UsdGeom.GetStageMetersPerUnit(stage)
    cache=UsdGeom.XformCache()
    rng=np.random.default_rng(20261103)
    report={'source':str(args.scene.resolve()),'units':'meters','coordinate_frame':'USD_world',
            'up_axis':str(UsdGeom.GetStageUpAxis(stage)), 'voxel_size_m':args.voxel,
            'colors':'height visualization; not texture RGB',
            'method':'area-weighted polygon surface sampling, then one observed point per occupied voxel',
            'excluded':['robot','invisible geometry','guide/proxy geometry','lights and cameras'],
            'notes':['Samples complete authored surfaces, including occluded/back faces.',
                     'Subdivision schemes are sampled as their polygon control meshes.',
                     'Coordinates retain the saved USD world origin; no ROS map-to-odom alignment is assumed.'],
            'geometry':[]}
    clouds=[]
    start=time.time()
    for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        path=str(prim.GetPath())
        if not path.startswith('/World/IC_building_scene/'):
            continue
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
            continue
        im=UsdGeom.Imageable(prim)
        if im.ComputeVisibility() == 'invisible' or im.ComputePurpose() not in ('default','render'):
            continue
        tri,area=geometry(prim,cache,unit)
        if not len(area): continue
        total=float(area.sum())
        n=max(1,int(np.ceil(total*3/(args.voxel**2))))
        cdf=np.cumsum(area)
        for first in range(0,n,500000):
            count=min(500000,n-first)
            chosen=np.searchsorted(cdf,rng.random(count)*cdf[-1])
            v=tri[chosen]
            u=np.sqrt(rng.random(count)); w=rng.random(count)
            xyz=((1-u[:,None])*v[:,0] + (u*(1-w))[:,None]*v[:,1]+(u*w)[:,None]*v[:,2])
            clouds.append(xyz.astype(np.float32))
        report['geometry'].append({'path':path,'area_m2':total,'samples':n,
                                   'triangles':len(tri)})
        if len(report['geometry'])%100==0:
            print('sampled',len(report['geometry']),'objects;',sum(len(x) for x in clouds),'points',flush=True)
    print('voxel filtering',flush=True)
    xyz=np.concatenate(clouds); del clouds
    report['samples_before_voxel']=len(xyz)
    cells=np.floor(xyz.astype(np.float64)/args.voxel).astype(np.int64)
    cells-=cells.min(axis=0)
    extent=cells.max(axis=0)+1
    assert np.prod(extent.astype(object)) < 2**63
    keys=(cells[:,0]*extent[1]+cells[:,1])*extent[2]+cells[:,2]
    del cells
    _,keep=np.unique(keys,return_index=True); del keys
    xyz=xyz[keep]; del keep
    assert np.isfinite(xyz).all()
    lo,hi=xyz.min(axis=0),xyz.max(axis=0)
    height=(xyz[:,2]-lo[2])/max(float(hi[2]-lo[2]),1e-6)
    palette=np.array([[30,60,200],[0,185,225],[65,210,100],[255,220,45],[225,55,35]])
    rgb=np.column_stack([np.interp(height,np.linspace(0,1,len(palette)),palette[:,i])
                         for i in range(3)]).astype(np.uint8)
    write_clouds(args.output,xyz,rgb)
    table=stage.GetPrimAtPath('/World/IC_building_scene/objects/table')
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeWorldBound(table).ComputeAlignedRange()
    report.update(point_count=len(xyz),bounds_min_m=lo.tolist(),bounds_max_m=hi.tolist(),
                  extent_m=(hi-lo).tolist(),surface_area_m2=sum(x['area_m2'] for x in report['geometry']),
                  reference_table_height_m=float(bounds.GetSize()[2]*unit),elapsed_s=time.time()-start)
    assert abs(report['reference_table_height_m']-.8)<1e-5
    (args.output/'scene_map.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    np.savez_compressed(args.output/'preview_samples.npz',xyz=xyz[::max(1,len(xyz)//250000)],
                        rgb=rgb[::max(1,len(xyz)//250000)])
    print(json.dumps({k:v for k,v in report.items() if k!='geometry'},ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    main()
