#include <cstdio>
#include <cstring>

#include "ValueHistory.h"
#include "imgui.h"

ValueHistory::ValueHistory() : m_hsamples(0) {
  for (int i = 0; i < MAX_HISTORY; ++i)
    m_samples[i] = 0;
}

float ValueHistory::getSampleMin() const {
  float val = m_samples[0];
  for (int i = 1; i < MAX_HISTORY; ++i)
    if (m_samples[i] < val)
      val = m_samples[i];
  return val;
}

float ValueHistory::getSampleMax() const {
  float val = m_samples[0];
  for (int i = 1; i < MAX_HISTORY; ++i)
    if (m_samples[i] > val)
      val = m_samples[i];
  return val;
}

float ValueHistory::getAverage() const {
  float val = 0;
  for (int i = 0; i < MAX_HISTORY; ++i)
    val += m_samples[i];
  return val / static_cast<float>(MAX_HISTORY);
}

void GraphParams::setRect(const int ix, const int iy, const int iw, const int ih, const int ipad) {
  x = ix;
  y = iy;
  w = iw;
  h = ih;
  pad = ipad;
}

void GraphParams::setValueRange(const float ivmin, const float ivmax, const int indiv, const char *iunits) {
  vmin = ivmin;
  vmax = ivmax;
  ndiv = indiv;
  strcpy_s(units, iunits);
}

void drawGraphBackground(const GraphParams *p) {
  // BG
  imguiDrawRoundedRect(static_cast<float>(p->x), static_cast<float>(p->y), static_cast<float>(p->w), static_cast<float>(p->h), static_cast<float>(p->pad), imguiRGBA(64, 64, 64, 128));

  const float sy = (p->h - p->pad * 2) / (p->vmax - p->vmin);
  const float oy = p->y + p->pad - p->vmin * sy;

  // Divider Lines
  for (int i = 0; i <= p->ndiv; ++i) {
    char text[64];
    const float u = static_cast<float>(i) / static_cast<float>(p->ndiv);
    const float v = p->vmin + (p->vmax - p->vmin) * u;
    std::snprintf(text, 64, "%.2f %s", v, p->units);
    const float fy = oy + v * sy;
    imguiDrawText(p->x + p->w - p->pad, static_cast<int>(fy) - 4, IMGUI_ALIGN_RIGHT, text, imguiRGBA(0, 0, 0, 255));
    imguiDrawLine(static_cast<float>(p->x) + static_cast<float>(p->pad), fy, static_cast<float>(p->x) + static_cast<float>(p->w) - static_cast<float>(p->pad) - 50, fy, 1.0f, imguiRGBA(0, 0, 0, 64));
  }
}

void drawGraph(const GraphParams *p, const ValueHistory *graph,
               const int idx, const char *label, const unsigned int col) {
  const float sx = (p->w - p->pad * 2) / static_cast<float>(graph->getSampleCount());
  const float sy = (p->h - p->pad * 2) / (p->vmax - p->vmin);
  const float ox = static_cast<float>(p->x) + static_cast<float>(p->pad);
  const float oy = static_cast<float>(p->y) + static_cast<float>(p->pad) - p->vmin * sy;

  // Values
  float px = 0, py = 0;
  for (int i = 0; i < graph->getSampleCount() - 1; ++i) {
    const float x = ox + i * sx;
    const float y = oy + graph->getSample(i) * sy;
    if (i > 0)
      imguiDrawLine(px, py, x, y, 2.0f, col);
    px = x;
    py = y;
  }

  // Label
  constexpr int size = 15;
  constexpr int spacing = 10;
  const int ix = p->x + p->w + 5;
  const int iy = p->y + p->h - (idx + 1) * (size + spacing);

  imguiDrawRoundedRect(static_cast<float>(ix), static_cast<float>(iy), static_cast<float>(size), static_cast<float>(size), 2.0f, col);

  char text[64];
  std::snprintf(text, 64, "%.2f %s", graph->getAverage(), p->units);
  imguiDrawText(ix + size + 5, iy + 3, IMGUI_ALIGN_LEFT, label, imguiRGBA(255, 255, 255, 192));
  imguiDrawText(ix + size + 150, iy + 3, IMGUI_ALIGN_RIGHT, text, imguiRGBA(255, 255, 255, 128));
}
