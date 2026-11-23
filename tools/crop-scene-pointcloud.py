"""Crop full-resolution scene points to the top slab's vertical footprint.

No points are resampled and original XYZ/RGB bytes are retained.
"""
import json
import argparse
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom


def read_ply(path):
    with path.open('rb') as f:
        lines=[]
        while True:
            line=f.readline()
            if not line:
                raise ValueError('Missing PLY header terminator')
            lines.append(line)
            if line == b'end_header\n': break
        offset=f.tell()
    assert b'format binary_little_endian 1.0\n' in lines
    count=int(next(x for x in lines if x.startswith(b'element vertex ')).split()[2])
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),
                    ('red','u1'),('green','u1'),('blue','u1')])
    assert path.stat().st_size == offset+count*dtype.itemsize
    return np.memmap(path,dtype=dtype,offset=offset,shape=(count,),mode='r')


def write_ply(path,data):
    with path.open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\n'
                 'comment units meters; frame USD_world; Z up\n'
                 'comment original 2cm points; crop only; no further downsampling\n'
                 'comment RGB encodes height\n'
                 f'element vertex {len(data)}\n'
                 'property float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode())
        data.tofile(f)


def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=root.parent/'data/exported-map/scene_map.ply')
    parser.add_argument('--scene',type=Path,default=root.parent/'runtime/localization_scene.local.usda')
    parser.add_argument('--output',type=Path,default=root.parent/'data/exported-map/cropped')
    args=parser.parse_args()
    source,scene,destination=args.source,args.scene,args.output
    destination.mkdir(exist_ok=True,parents=True)
    stage=Usd.Stage.Open(str(scene))
    prim=stage.GetPrimAtPath('/World/IC_building_scene/top')
    vertices=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),dtype=np.float64)
    matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim))
    vertices=(vertices@matrix[:3,:3]+matrix[3,:3])*UsdGeom.GetStageMetersPerUnit(stage)
    lo,hi=vertices.min(axis=0),vertices.max(axis=0)
    # This scene's slab is an axis-aligned cuboid; verify before using XY bounds.
    assert len(np.unique(vertices,axis=0))==8
    assert np.all(np.isclose(vertices,lo,atol=1e-8)|np.isclose(vertices,hi,atol=1e-8))
    original=read_ply(source)
    eps=1e-5
    inside=((original['x']>=lo[0]-eps)&(original['x']<=hi[0]+eps)
            &(original['y']>=lo[1]-eps)&(original['y']<=hi[1]+eps))
    cropped=original[inside]
    write_ply(destination/'房间范围_2cm_保留顶板.ply',cropped)
    # Hide only the slab's six surfaces, rather than chopping all high objects.
    xyz=np.column_stack([cropped[k] for k in ['x','y','z']]).astype(np.float64)
    in_slab=np.all((xyz>=lo-eps)&(xyz<=hi+eps),axis=1)
    on_slab=np.any((np.abs(xyz-lo)<=eps)|(np.abs(xyz-hi)<=eps),axis=1)
    open_view=cropped[~(in_slab&on_slab)]
    write_ply(destination/'房间范围_2cm_去顶板查看版.ply',open_view)
    # Verify complete removal of the exterior footprint and exact preservation.
    assert np.array_equal(cropped,original[inside])
    assert cropped['x'].min()>=lo[0]-eps and cropped['x'].max()<=hi[0]+eps
    assert cropped['y'].min()>=lo[1]-eps and cropped['y'].max()<=hi[1]+eps
    checked=read_ply(destination/'房间范围_2cm_保留顶板.ply')
    assert np.array_equal(checked,cropped)
    assert np.array_equal(read_ply(destination/'房间范围_2cm_去顶板查看版.ply'),open_view)
    report={'source':str(source),'roof_prim':str(prim.GetPath()),
            'units':'meters','coordinate_frame':'USD_world','voxel_size_m':.02,
            'operation':'vertical footprint crop only; no resampling',
            'footprint_min_xy_m':lo[:2].tolist(),'footprint_max_xy_m':hi[:2].tolist(),
            'roof_z_min_max_m':[float(lo[2]),float(hi[2])],
            'original_points':len(original),'cropped_points':len(cropped),
            'removed_exterior_points':int((~inside).sum()),
            'open_roof_points':len(open_view),'roof_surface_points_removed':len(cropped)-len(open_view),
            'verification':'exact source point bytes retained; PLY reread equality passed',
            'roof_removal_tolerance_m':eps}
    (destination/'裁剪记录.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__=='__main__': main()
