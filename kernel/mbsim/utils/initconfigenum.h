/* Copyright (C) 2004-2012 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef INITCONFIGENUM_H_
#define INITCONFIGENUM_H_

#include <unordered_set>
#include <functional>
#include <cassert>

namespace MBSim {

class InitConfigEnum {
  public:
    bool operator==(const InitConfigEnum& a) const { return this==&a; }
    bool operator!=(const InitConfigEnum& a) const { return this!=&a; }

    InitConfigEnum() {}
    ~InitConfigEnum() {}
    // must be defined due to SWIG
    // static void* operator new(std::size_t);
    // static void operator delete(void*, std::size_t);
  private:
    static void operator delete[](void*, std::size_t);
    static void* operator new[](std::size_t);
    InitConfigEnum(const InitConfigEnum&)=delete;
    InitConfigEnum(InitConfigEnum&&)=delete;
    InitConfigEnum& operator=(const InitConfigEnum&)=delete;
    InitConfigEnum& operator=(InitConfigEnum&&)=delete;
};

#ifndef SWIG

struct InitConfigEnumOp {
  size_t operator()(const InitConfigEnum &f) const { return reinterpret_cast<size_t>(&f); }
  bool operator()(const InitConfigEnum &a, const InitConfigEnum &b) const { return &a==&b; }
};

typedef std::unordered_set<std::reference_wrapper<const InitConfigEnum>, InitConfigEnumOp, InitConfigEnumOp> InitConfigSet;

#endif

}

#endif
