//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#pragma once
#include <cstdint>

class dtProximityGrid {
  float m_cellSize{};
  float m_invCellSize{};

  struct Item {
    uint16_t id{};
    short x{};
    short y{};
    uint16_t next{};
  };
  Item *m_pool{};
  int m_poolHead{};
  int m_poolSize{};

  uint16_t *m_buckets{};
  int m_bucketsSize{};

  int m_bounds[4]{};

public:
  dtProximityGrid() = default;
  ~dtProximityGrid();

  bool init(int poolSize, float cellSize);

  void clear();

  void addItem(uint16_t id,
               float minx, float miny,
               float maxx, float maxy);

  int queryItems(float minx, float miny,
                 float maxx, float maxy,
                 uint16_t *ids, int maxIds) const;

  int getItemCountAt(int x, int y) const;

  const int *getBounds() const { return m_bounds; }
  float getCellSize() const { return m_cellSize; }

  // Explicitly disabled copy constructor and copy assignment operator.
  dtProximityGrid(const dtProximityGrid &) = delete;
  dtProximityGrid &operator=(const dtProximityGrid &) = delete;
};

dtProximityGrid *dtAllocProximityGrid();
void dtFreeProximityGrid(dtProximityGrid *ptr);
