/**
 * Authors: shima-529, Alexander Chepkov
 *
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 */

module cm3.bitop;

public import core.bitop;
import core.volatile : volatileLoad, volatileStore;
import std.traits : isNumeric;

extern (C):
@nogc:
nothrow:

void st(T)(ref T reg, T val) if (isNumeric!(T)) =>
	volatileStore(&reg, val);

T ld(T)(ref T reg) if (isNumeric!(T)) =>
	volatileLoad(&reg);

ref bset(T, U)(ref T reg, U[] valarr...) if (isNumeric!(T) && isNumeric!(U)) {
	T val;
	foreach (arg; valarr)
		val |= (1 << arg);
	st(reg, ld(reg) | val);
	return reg;
}

ref bclr(T, U)(ref T reg, U[] valarr...) if (isNumeric!(T) && isNumeric!(U)) {
	T val;
	foreach (arg; valarr)
		val |= (1 << arg);
	st(reg, ld(reg) & ~val);
	return reg;
}

bool bget(T, U)(ref T reg, U pos) if (isNumeric!(T) && isNumeric!(U)) =>
	((ld(reg) & (1 << pos)) != 0);
