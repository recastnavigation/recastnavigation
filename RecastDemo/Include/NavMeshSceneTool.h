#ifndef __MYNAVMESHSCENE_H__
#define __MYNAVMESHSCENE_H__

#include "Sample.h"
#include <scene.h>
#include <agent.h>
#include <stdio.h>
#include <memory>
#include "Sample.h"
#include <cmath>
#include <vector>

class Player : public NavMeshScene::Agent
{
public:

    void OnHit(Agent* agent) override
    {
        if (!mRobot)
        {
            return;
        }
        changeDir();
    }

public:

    void changeDir()
    {
        float angle = float(rand() % 360);
        float vx = cos(3.14f * angle / 180);
        float vy = -sin(3.14f * angle / 180);
        float s = sqrt(vx*vx + vy*vy);
        vx = vx / s;
        vy = vy / s;
        float v[3] = { vx * 5, 0, vy * 5 };
        SetVelocity(v);
    }

    bool mRobot = true;
};

class NavMeshSceneTool : public SampleTool
{
    Sample* m_sample;

public:
    NavMeshSceneTool();

    virtual int type() { return TOOL_MY_NAV_MESH_SCENE; }
    virtual void init(Sample* sample);
    virtual void reset();
    virtual void handleMenu();
    virtual void handleClick(const float* s, const float* p, bool shift);
    virtual void handleToggle();
    virtual void handleStep();
    virtual void handleUpdate(const float dt);
    virtual void handleRender();
    virtual void handleRenderOverlay(double* proj, double* model, int* view);

private:
    void drawAgent(const float* pos, float r, float h, float c, const unsigned int col);
    void doInit();

    std::shared_ptr<NavMeshScene::Scene> mScene;
    std::vector<std::shared_ptr<Player>> mAgents;
    int mMeshMode;
    int mCurrentAgent;
};

#endif
