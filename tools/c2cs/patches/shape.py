# Manual fixups applied after converting shape.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# const void* geometry becomes the shape geometry union
s = s.replace('in ShapeDef def, object? geometry, ShapeType shapeType )', 'in ShapeDef def, in ShapeUnion geometry, ShapeType shapeType )')
for t, f in (('Capsule', 'capsule'), ('Circle', 'circle'), ('Polygon', 'polygon'), ('Segment', 'segment'),
             ('ChainSegment', 'chainSegment')):
    s = s.replace('shape.%s = *(const %s*)geometry;' % (f, t), 'shape.%s = geometry.%s;' % (f, f))
s = s.replace('CreateShape( bodyId, def, circle, ShapeType.circleShape )', 'CreateShape( bodyId, def, new ShapeUnion { circle = circle }, ShapeType.circleShape )')
s = s.replace('CreateShape( bodyId, def, capsule, ShapeType.capsuleShape )', 'CreateShape( bodyId, def, new ShapeUnion { capsule = capsule }, ShapeType.capsuleShape )')
s = s.replace('CreateShape( bodyId, def, polygon, ShapeType.polygonShape )', 'CreateShape( bodyId, def, new ShapeUnion { polygon = polygon }, ShapeType.polygonShape )')
s = s.replace('CreateShape( bodyId, def, segment, ShapeType.segmentShape )', 'CreateShape( bodyId, def, new ShapeUnion { segment = segment }, ShapeType.segmentShape )')
s = s.replace('CreateShape( bodyId, def, ref local, ShapeType.chainSegmentShape )', 'CreateShape( bodyId, def, new ShapeUnion { chainSegment = local }, ShapeType.chainSegmentShape )')
s = s.replace('CreateShapeInternal( world, body, transform, ref shapeDef, ref chainSegment, ShapeType.chainSegmentShape )',
              'CreateShapeInternal( world, ref body, transform, shapeDef, new ShapeUnion { chainSegment = chainSegment }, ShapeType.chainSegmentShape )')
open(p, 'w', encoding='utf-8', newline='\n').write(s)
