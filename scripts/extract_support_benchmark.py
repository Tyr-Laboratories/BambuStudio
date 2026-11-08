#!/usr/bin/env python3
from pathlib import Path
import argparse,hashlib,io,json,zipfile,xml.etree.ElementTree as ET
parser=argparse.ArgumentParser(description="Extract one saved object for reproducible support benchmarks, preserving its mesh and paint bytes.")
parser.add_argument("source",type=Path)
parser.add_argument("output",type=Path)
parser.add_argument("--object-id",required=True)
parser.add_argument("--plate",required=True)
parser.add_argument("--offset-x",type=float,default=0.)
parser.add_argument("--offset-y",type=float,default=0.)
args=parser.parse_args()
source=args.source
out=args.output
if out.resolve()==source.resolve() or out.exists():
 parser.error('output must be a new file, distinct from the source project')
out.parent.mkdir(parents=True,exist_ok=True)
core='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
with zipfile.ZipFile(source) as z:
 raw=z.read('3D/3dmodel.model')
 for event,pair in ET.iterparse(io.BytesIO(raw),events=('start-ns',)):
  prefix,uri=pair
  if prefix not in ('xml','xmlns'):ET.register_namespace(prefix,uri)
 main=ET.fromstring(raw);resources=main.find('{'+core+'}resources');build=main.find('{'+core+'}build')
 for obj in list(resources):
  if obj.tag.endswith('object') and obj.get('id')!=args.object_id:resources.remove(obj)
 model_files={e.get('{http://schemas.microsoft.com/3dmanufacturing/production/2015/06}path').lstrip('/')
              for e in resources.iter() if e.get('{http://schemas.microsoft.com/3dmanufacturing/production/2015/06}path')}
 if not any(o.get('id')==args.object_id for o in resources):
  raise ValueError('Object id not found')
 for item in list(build):
  if item.get('objectid')!=args.object_id:build.remove(item)
  else:
   t=[float(v) for v in item.get('transform').split()];t[9]+=args.offset_x;t[10]+=args.offset_y
   item.set('transform',' '.join(format(v,'.12g') for v in t))
 settings=ET.fromstring(z.read('Metadata/model_settings.config'))
 for e in list(settings):
  if e.tag=='object' and e.get('id')!=args.object_id:settings.remove(e)
  elif e.tag=='plate':
   pid=next((x for x in e if x.get('key')=='plater_id'),None)
   if pid is None or pid.get('value')!=args.plate:settings.remove(e);continue
   pid.set('value','1')
   for x in list(e):
    if x.tag=='model_instance' and not any(a.get('key')=='object_id' and a.get('value')==args.object_id for a in x):e.remove(x)
  elif e.tag=='assemble':settings.remove(e)
 with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=1) as dest:
  for name in ['[Content_Types].xml','_rels/.rels','Metadata/project_settings.config',*sorted(model_files)]:
   dest.writestr(name,z.read(name))
  dest.writestr('3D/3dmodel.model',ET.tostring(main,encoding='utf-8',xml_declaration=True))
  dest.writestr('Metadata/model_settings.config',ET.tostring(settings,encoding='utf-8',xml_declaration=True))
  # Include only the remaining external object relationship.
  rel='3D/_rels/3dmodel.model.rels'
  if rel in z.namelist():
   tree=ET.fromstring(z.read(rel))
   for x in list(tree):
    if x.get('Target','').lstrip('/') not in model_files:tree.remove(x)
   ET.register_namespace('', 'http://schemas.openxmlformats.org/package/2006/relationships')
   dest.writestr(rel,ET.tostring(tree,encoding='utf-8',xml_declaration=True))
manifest={'original_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'fixture_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'object_id':args.object_id,'original_plate':args.plate,'translation_xy_mm':[args.offset_x,args.offset_y],'mesh_and_paint_preserved':True}
out.with_suffix('.provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(manifest)
