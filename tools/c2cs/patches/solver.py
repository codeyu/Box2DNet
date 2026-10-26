# Manual fixups applied after converting solver.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# graph color blocks are offsets into the graph block array
s = s.replace('\t\t\tSyncBlock* graphColorBlocks[GraphColorCount] = { 0 };',
              '\t\t\tSpan<int> graphColorBlocks = stackalloc int[GraphColorCount];')
open(p, 'w', encoding='utf-8', newline='\n').write(s)
