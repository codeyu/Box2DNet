# Manual fixups applied after converting broad_phase.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# LSD radix sort, written by hand because it swaps pointers between two buffers
a = s.index('\tinternal static void RadixSortKeys(')
b = s.index('\n\t}\n', a) + 4
s = s[:a] + '''\tinternal static void RadixSortKeys( Span<ulong> keys, Span<ulong> tempKeys, int count )
	{
		Span<int> digitCounts = stackalloc int[8 * 256];
		digitCounts.Clear();
		for ( int i = 0; i < count; ++i )
		{
			ulong key = keys[i];
			for ( int digit = 0; digit < 8; ++digit )
			{
				digitCounts[digit * 256 + (int)( ( key >> ( 8 * digit ) ) & 0xFF )] += 1;
			}
		}

		Span<ulong> source = keys;
		Span<ulong> target = tempKeys;
		bool sourceIsKeys = true;

		for ( int digit = 0; digit < 8; ++digit )
		{
			int shift = 8 * digit;
			Span<int> offsets = digitCounts.Slice( digit * 256, 256 );

			if ( offsets[(int)( ( source[0] >> shift ) & 0xFF )] == count )
			{
				// All keys have the same value for this digit. Common for shape pair keys.
				continue;
			}

			int sum = 0;
			for ( int i = 0; i < 256; ++i )
			{
				int digitCount = offsets[i];
				offsets[i] = sum;
				sum += digitCount;
			}

			for ( int i = 0; i < count; ++i )
			{
				ulong key = source[i];
				target[offsets[(int)( ( key >> shift ) & 0xFF )]++] = key;
			}

			// Swap for the next pass
			Span<ulong> swap = source;
			source = target;
			target = swap;
			sourceIsKeys = !sourceIsKeys;
		}

		if ( sourceIsKeys == false )
		{
			source.Slice( 0, count ).CopyTo( keys );
		}
	}
''' + s[b:]
open(p, 'w', encoding='utf-8', newline='\n').write(s)
