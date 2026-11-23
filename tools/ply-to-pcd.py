"""Convert this project's XYZ/RGB binary PLY files to binary PCD, without resampling."""
import argparse
import json
from pathlib import Path

import numpy as np


def convert(source):
    with source.open('rb') as f:
        header=[]
        while True:
            line=f.readline()
            if not line:
                raise ValueError('Incomplete PLY header')
            header.append(line)
            if line.strip()==b'end_header': break
        offset=f.tell()
    assert b'format binary_little_endian 1.0\n' in header
    assert [x.strip() for x in header if x.startswith(b'property ')]==[
        b'property float x',b'property float y',b'property float z',
        b'property uchar red',b'property uchar green',b'property uchar blue']
    count=int(next(x for x in header if x.startswith(b'element vertex ')).split()[2])
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
    assert source.stat().st_size==offset+count*dtype.itemsize
    points=np.memmap(source,mode='r',offset=offset,dtype=dtype,shape=(count,))
    target=source.with_suffix('.pcd')
    temporary=target.with_suffix('.pcd.partial')
    pcd_header=(
        '# .PCD v0.7 - Point Cloud Data file format\n'
        '# units meters; frame USD_world; Z up; RGB encodes height\n'
        'VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n'
        f'COUNT 1 1 1 1\nWIDTH {count}\nHEIGHT 1\n'
        f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n').encode('ascii')
    step=1000000
    with temporary.open('wb') as f:
        f.write(pcd_header)
        for start in range(0,count,step):
            part=points[start:start+step]
            packed=np.empty((len(part),4),dtype='<f4')
            for i,key in enumerate(('x','y','z')): packed[:,i]=part[key]
            packed[:,3].view('<u4')[:]=(part['red'].astype(np.uint32)<<16)|(part['green'].astype(np.uint32)<<8)|part['blue']
            packed.tofile(f)
    assert temporary.stat().st_size==len(pcd_header)+count*16
    result=np.memmap(temporary,mode='r',offset=len(pcd_header),dtype='<f4',shape=(count,4))
    for start in range(0,count,step):
        a=points[start:start+step]; b=result[start:start+step]
        for i,key in enumerate(('x','y','z')):
            assert np.array_equal(a[key].view('<u4'),b[:,i].view('<u4'))
        colors=b[:,3].view('<u4')
        for shift,key in ((16,'red'),(8,'green'),(0,'blue')):
            assert np.array_equal(((colors>>shift)&255).astype(np.uint8),a[key])
    del result
    temporary.replace(target)
    report={'source':str(source.resolve()),'output':str(target.resolve()),'points':count,
            'bytes':target.stat().st_size,'format':'PCD v0.7 binary XYZ RGB',
            'units':'meters','frame':'USD_world','voxel_size_m':.02,
            'validation':'all XYZ float bits and all RGB bytes match the source; no resampling'}
    target.with_suffix('.pcd.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sources',type=Path,nargs='+')
    for source in parser.parse_args().sources: convert(source)
