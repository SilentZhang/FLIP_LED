#include "SandSim.h"
#include "charlie.h"

uint8_t screen[32];
_iq invertSpacing;

#define FLUID_CELL 0U
#define AIR_CELL 1U
#define SOLID_CELL 2U

_iq particlePos[NumberOfParticles * 2];
_iq particleVel[NumberOfParticles * 2];

static _iq uVel[CellCount];
static _iq vVel[CellCount];
static _iq uPrev[CellCount];
static _iq vPrev[CellCount];
static _iq uWeight[CellCount];
static _iq vWeight[CellCount];

//static _iq pressure[CellCount];
static _iq particleDensity[CellCount];
static _iq solidMask[CellCount];
static uint8_t cellType[CellCount];
static _iq particleRestDensity = 0;

// 心形内部区域（红色格，从图上按 15 列 x 16 行提取），每行 bit14..bit0 对应 x=1..15
static const uint16_t heartInterior[16] = {
    0x0000, 0x0000, 0x0C18, 0x1E3C, 0x3F7E, 0x7FFF, 0x7FFF, 0x7FFF,
    0x3FFE, 0x1FFC, 0x0FF8, 0x07F0, 0x03E0, 0x01C0, 0x0080, 0x0000
};

// 心形墙壁（蓝色格），用于点亮轮廓
static const uint16_t heartWall[16] = {
    0x0000, 0x1E3C, 0x3266, 0x61C3, 0x4081, 0x0000, 0x0000, 0x0000,
    0x4001, 0x6003, 0x3006, 0x180C, 0x0C18, 0x0630, 0x0360, 0x01C0
};

// 沙漏内部区域：上下两个漏斗形腔体，中间 y=8/9 行仅 x=8 一格为颈部通道
static const uint16_t hourglassInterior[16] = {
    0x0000, 0x1FFC, 0x0FF8, 0x07F0, 0x03E0, 0x01C0, 0x01C0, 0x0080,
    0x0080, 0x01C0, 0x01C0, 0x03E0, 0x07F0, 0x0FF8, 0x1FFC, 0x0000
};

// 沙漏墙壁（轮廓），用于点亮外形
static const uint16_t hourglassWall[16] = {
    0x3FFE, 0x2002, 0x1004, 0x0808, 0x0410, 0x0220, 0x0220, 0x0140,
    0x0140, 0x0220, 0x0220, 0x0410, 0x0808, 0x1004, 0x2002, 0x3FFE
};

static uint8_t wallMode = WALL_RECT;

// ---- 磁流体模式参数 ----
#define FERRO_PARTICLES  56U           // 磁流体粒子数（约5x5灯格的一团）
#define FERRO_CYCLE      120U          // 一个“聚合-冲撞-爆散”周期的帧数(约1.3秒)
#define FERRO_MERGE      _IQ(20.0f)    // 向质心收拢的弹性系数(1/s^2)
#define FERRO_STIR       _IQ(8.0f)     // 绕质心搅动的加速度(1/s^2)
#define FERRO_DRIVE      _IQ(5.0f)     // 冲向墙壁的整体加速度(m/s^2)
#define FERRO_DAMP       _IQ(0.96f)    // 每帧速度阻尼
#define FERRO_BURST      _IQ(1.8f)     // 撞墙爆散瞬间的喷射速度(m/s)

static uint8_t ferroActive = 0;
static uint16_t ferroFrame = 0;
static uint8_t ferroDir = 0;

// 当前形状掩码，为0表示矩形（全部内部、无轮廓墙）
static const uint16_t *shapeInterior = 0;
static const uint16_t *shapeWall = 0;

static _iq minX = Spacing + ParticleRadius;
static _iq minY = Spacing + ParticleRadius;
static _iq maxX;
static _iq maxY;

#define CellPrefixCountSize (CellCount + 1U)

static uint8_t cellParticleCountPrefix[CellPrefixCountSize];
static uint8_t particlePosId[NumberOfParticles];

// 当前参与仿真的粒子数：心形容器容积小，用一半粒子以获得可见的液面
static unsigned int activeParticles = NumberOfParticles;

void printLocation(unsigned int n);

// 该网格是否可容纳流体（非墙壁）
static uint8_t is_fluid_cell(unsigned int x, unsigned int y)
{
    if (x < 1U || x > CellNumX - 2U || y < 1U || y > CellNumY - 2U) {
        return 0U;
    }
    if (shapeInterior == 0) {
        return 1U;
    }
    return (uint8_t)((shapeInterior[y - 1U] >> (15U - x)) & 1U);
}

// 该网格是否是需要点亮的墙壁（形状轮廓）
static uint8_t is_visible_wall(unsigned int x, unsigned int y)
{
    if (shapeWall == 0) {
        return 0U;
    }
    if (x < 1U || x > CellNumX - 2U || y < 1U || y > CellNumY - 2U) {
        return 0U;
    }
    return (uint8_t)((shapeWall[y - 1U] >> (15U - x)) & 1U);
}

static int clamp_index(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static _iq clampf_local(_iq value, _iq min_value, _iq max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// 简易线性同余随机数，返回 [0,1)，用于粒子初始位置抖动
static uint32_t rngState = 12345U;

static _iq rand_unit(void)
{
    rngState = rngState * 1664525U + 1013904223U;
    return _IQ((float)(rngState >> 8) / 16777216.0f);
}

static void setup_solid_mask(void) {
    for (unsigned int x = 0; x < CellNumX; x++) {
        for (unsigned int y = 0; y < CellNumY; y++) {
            solidMask[INDEX(x, y)] = is_fluid_cell(x, y) ? _IQ(1.0) : _IQ(0.0);
        }
    }
}

void InitParticles() {
    memset(particleVel, 0, sizeof(particleVel));
    memset(uVel, 0, sizeof(uVel));
    memset(vVel, 0, sizeof(vVel));
    memset(uPrev, 0, sizeof(uPrev));
    memset(vPrev, 0, sizeof(vPrev));
    //memset(pressure, 0, sizeof(pressure));
    memset(particleDensity, 0, sizeof(particleDensity));
    particleRestDensity = _IQ(0.0);
    setup_solid_mask();



    const _iq h = Spacing;
    const _iq r = ParticleRadius;
    invertSpacing = _IQdiv(_IQ(1.0), Spacing);
    maxX = _IQmpy(_IQ(CellNumX - 1U), Spacing) - ParticleRadius;
    maxY = _IQmpy(_IQ(CellNumY - 1U), Spacing) - ParticleRadius;

    const _iq half = _IQ(0.5);
    const _iq jitter = _IQmpy(Spacing, _IQ(0.3));
    unsigned int p_num = 0;

    // 矩形/心形自下而上逐格布置，保证静止时流体堆在容器底部；
    // 沙漏自上而下布置，让沙子先聚在上腔，切换后能看到向下漏沙
    if (ferroActive) {
        // 磁流体：在画面中央摆成一团，随后靠聚合力收成球状
        for (unsigned int i = 0; i < activeParticles; i++) {
            int x = 4 + (int)(i % 8U);
            int y = 6 + (int)(i / 8U);
            _iq px = _IQmpy(_IQ(x) + half, h);
            _iq py = _IQmpy(_IQ(y) + half, h);
            _iq ox = _IQmpy(jitter, _IQmpy(_IQ(2.0), rand_unit()) - _IQ(1.0));
            _iq oy = _IQmpy(jitter, _IQmpy(_IQ(2.0), rand_unit()) - _IQ(1.0));
            particlePos[XID(i)] = px + ox;
            particlePos[YID(i)] = py + oy;
            p_num++;
        }
    }
    else for (unsigned int pass = 0; pass < 2U && p_num < activeParticles; pass++) {
        for (unsigned int row = 0; row <= CellNumY - 3U && p_num < activeParticles; row++) {
            int y = (wallMode == WALL_HOURGLASS) ? (int)row + 1 : (int)(CellNumY - 2U - row);
            for (int x = 1; x <= (int)CellNumX - 2 && p_num < activeParticles; x++) {
                if (!is_fluid_cell((unsigned int)x, (unsigned int)y)) {
                    continue;
                }
                _iq px = _IQmpy(_IQ(x) + half, h);
                _iq py = _IQmpy(_IQ(y) + half, h);
                _iq ox = _IQmpy(jitter, _IQmpy(_IQ(2.0), rand_unit()) - _IQ(1.0));
                _iq oy = _IQmpy(jitter, _IQmpy(_IQ(2.0), rand_unit()) - _IQ(1.0));
                particlePos[XID(p_num)] = px + ox;
                particlePos[YID(p_num)] = py + oy;
                p_num++;
            }
        }
    }

    for (; p_num < NumberOfParticles; p_num++) {
        particlePos[XID(p_num)] = h + r;
        particlePos[YID(p_num)] = h + r;
    }
}

void SandSim_SetWallMode(uint8_t mode)
{
    if (mode == wallMode) {
        return;
    }
    wallMode = mode;
    ferroActive = 0;

    switch (wallMode) {
    case WALL_HEART:
        shapeInterior = heartInterior;
        shapeWall = heartWall;
        // 心形容积约为矩形的一半，减少粒子数以留出可见液面
        activeParticles = 64U;
        break;
    case WALL_HOURGLASS:
        shapeInterior = hourglassInterior;
        shapeWall = hourglassWall;
        // 上腔38格，用36个粒子装满上腔，漏完后正好铺满下腔
        activeParticles = 36U;
        break;
    case WALL_FERRO:
        // 磁流体使用四周矩形墙壁，不点亮轮廓，只显示那团液体
        shapeInterior = 0;
        shapeWall = 0;
        activeParticles = FERRO_PARTICLES;
        ferroActive = 1;
        ferroFrame = 0;
        ferroDir = 3U;    // 第一个周期先朝下撞
        break;
    default:
        shapeInterior = 0;
        shapeWall = 0;
        activeParticles = NumberOfParticles;
        break;
    }

    // 容器改变后重新布置粒子，避免粒子被留在新墙壁内部
    InitParticles();
}

// 粒子若进入墙壁网格，则移动到最近的流体网格中心并清空速度
static void ConstrainParticlesToFluid(void)
{
    const _iq half = _IQ(0.5);

    for (unsigned int i = 0; i < activeParticles; i++) {
        _iq px = particlePos[XID(i)];
        _iq py = particlePos[YID(i)];
        int xi = clamp_index(_IQint(_IQmpy(px, invertSpacing)), 0, (int)CellNumX - 1);
        int yi = clamp_index(_IQint(_IQmpy(py, invertSpacing)), 0, (int)CellNumY - 1);
        if (is_fluid_cell((unsigned int)xi, (unsigned int)yi)) {
            continue;
        }

        int bestX = -1;
        int bestY = -1;
        _iq bestD2 = 0;
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                int nx = xi + dx;
                int ny = yi + dy;
                if (!is_fluid_cell((unsigned int)nx, (unsigned int)ny)) {
                    continue;
                }
                _iq cx = _IQmpy(_IQ(nx) + half, Spacing);
                _iq cy = _IQmpy(_IQ(ny) + half, Spacing);
                _iq ddx = cx - px;
                _iq ddy = cy - py;
                _iq d2 = _IQmpy(ddx, ddx) + _IQmpy(ddy, ddy);
                if (bestX < 0 || d2 < bestD2) {
                    bestD2 = d2;
                    bestX = nx;
                    bestY = ny;
                }
            }
        }

        if (bestX >= 0) {
            particlePos[XID(i)] = _IQmpy(_IQ(bestX) + half, Spacing);
            particlePos[YID(i)] = _IQmpy(_IQ(bestY) + half, Spacing);
            particleVel[XID(i)] = _IQ(0.0);
            particleVel[YID(i)] = _IQ(0.0);
        }
    }
}

// 磁流体：整团粒子被“磁力”收拢成球，随后被推向一面墙，撞墙瞬间爆散成几块，
// 再靠收拢力重新融合，如此循环。此模式不参与网格压力求解，纯粒子运动。
static void FerroIntegrate(void)
{
    const _iq count = _IQ((float)activeParticles);
    _iq cx = _IQ(0.0);
    _iq cy = _IQ(0.0);
    for (unsigned int i = 0; i < activeParticles; i++) {
        cx += particlePos[XID(i)];
        cy += particlePos[YID(i)];
    }
    cx = _IQdiv(cx, count);
    cy = _IQdiv(cy, count);

    uint16_t phase = ferroFrame % FERRO_CYCLE;
    if (phase == 0U) {
        ferroDir = (uint8_t)((ferroDir + 1U) & 3U);   // 每个周期换一面墙
    }
    ferroFrame++;

    // 冲撞阶段：整团朝目标墙加速
    _iq drvX = _IQ(0.0);
    _iq drvY = _IQ(0.0);
    if (phase >= FERRO_CYCLE / 2U && phase < FERRO_CYCLE - 20U) {
        switch (ferroDir) {
        case 0U: drvX = -FERRO_DRIVE; break;
        case 1U: drvY = -FERRO_DRIVE; break;
        case 2U: drvX =  FERRO_DRIVE; break;
        default: drvY =  FERRO_DRIVE; break;
        }
    }

    // 撞墙瞬间：沿半径向外喷射并叠加随机扰动，让一团分裂成几块
    if (phase == FERRO_CYCLE - 20U) {
        for (unsigned int i = 0; i < activeParticles; i++) {
            _iq dx = particlePos[XID(i)] - cx;
            _iq dy = particlePos[YID(i)] - cy;
            _iq d = _IQsqrt(_IQmpy(dx, dx) + _IQmpy(dy, dy));
            _iq ux;
            _iq uy;
            if (d > _IQ(0.001f)) {
                ux = _IQdiv(dx, d);
                uy = _IQdiv(dy, d);
            } else {
                ux = _IQmpy(_IQ(2.0f), rand_unit()) - _IQ(1.0f);
                uy = _IQmpy(_IQ(2.0f), rand_unit()) - _IQ(1.0f);
            }
            // 随机切向分量 = 径向向量旋转90度
            _iq tx = -uy;
            _iq ty =  ux;
            _iq k = _IQmpy(_IQ(2.0f), rand_unit()) - _IQ(1.0f);
            _iq vx = _IQmpy(FERRO_BURST, ux + _IQmpy(_IQ(0.8f), _IQmpy(k, tx)));
            _iq vy = _IQmpy(FERRO_BURST, uy + _IQmpy(_IQ(0.8f), _IQmpy(k, ty)));
            particleVel[XID(i)] += vx;
            particleVel[YID(i)] += vy;
        }
    }

    for (unsigned int i = 0; i < activeParticles; i++) {
        _iq px = particlePos[XID(i)];
        _iq py = particlePos[YID(i)];
        _iq dx = cx - px;
        _iq dy = cy - py;

        // 收拢力(指向质心) + 搅动力(绕质心旋转) + 冲撞驱动
        _iq ax = _IQmpy(FERRO_MERGE, dx) - _IQmpy(FERRO_STIR, dy) + drvX;
        _iq ay = _IQmpy(FERRO_MERGE, dy) + _IQmpy(FERRO_STIR, dx) + drvY;

        _iq vx = particleVel[XID(i)] + _IQmpy(ax, dt);
        _iq vy = particleVel[YID(i)] + _IQmpy(ay, dt);
        vx = _IQmpy(vx, FERRO_DAMP);
        vy = _IQmpy(vy, FERRO_DAMP);

        px += _IQmpy(vx, dt);
        py += _IQmpy(vy, dt);

        if (px < minX) {
            px = minX;
            vx = _IQmpy(vx, BOUNCYNESS);
        }
        if (px > maxX) {
            px = maxX;
            vx = _IQmpy(vx, BOUNCYNESS);
        }
        if (py < minY) {
            py = minY;
            vy = _IQmpy(vy, BOUNCYNESS);
        }
        if (py > maxY) {
            py = maxY;
            vy = _IQmpy(vy, BOUNCYNESS);
        }

        particlePos[XID(i)] = px;
        particlePos[YID(i)] = py;
        particleVel[XID(i)] = vx;
        particleVel[YID(i)] = vy;
    }
}

void ParticleIntegrate(_iq xAcceleration, _iq yAcceleration) {

    if (ferroActive) {
        FerroIntegrate();   // 磁流体模式自带驱动力，不跟随设备姿态
        return;
    }


    for (unsigned int i = 0; i < activeParticles; i++) {
        particleVel[XID(i)] += _IQmpy(xAcceleration, dt);
        particleVel[YID(i)] += _IQmpy(yAcceleration, dt);
        particlePos[XID(i)] += _IQmpy(particleVel[XID(i)], dt);
        particlePos[YID(i)] += _IQmpy(particleVel[YID(i)], dt);

        _iq x = particlePos[XID(i)];
        _iq y = particlePos[YID(i)];

        if (x < minX) {
            x = minX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (x > maxX) {
            x = maxX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (y < minY) {
            y = minY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)] ,BOUNCYNESS);
        }
        if (y > maxY) {
            y = maxY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)] ,BOUNCYNESS);
        }

        particlePos[XID(i)] = x;
        particlePos[YID(i)] = y;
    }

    ConstrainParticlesToFluid();
}

void PushParticlesApart(unsigned int nIters) {
    memset(cellParticleCountPrefix, 0, sizeof(cellParticleCountPrefix));
    memset(particlePosId, 0, sizeof(particlePosId));

    for (unsigned int i = 0; i < activeParticles; i++) {
        unsigned int xi = (unsigned int)clamp_index(_IQint(_IQmpy(particlePos[XID(i)], invertSpacing)), 0, (int)CellNumX - 1);
        unsigned int yi = (unsigned int)clamp_index(_IQint(_IQmpy(particlePos[YID(i)], invertSpacing)), 0, (int)CellNumY - 1);
        cellParticleCountPrefix[INDEX(xi, yi)]++;
    }

    unsigned int prefix = 0;
    for (unsigned int i = 0; i < CellCount; i++) {
        prefix += cellParticleCountPrefix[i];
        cellParticleCountPrefix[i] = prefix;
    }
    cellParticleCountPrefix[CellCount] = prefix;

    for (unsigned int i = 0; i < activeParticles; i++) {
        unsigned int xi = (unsigned int)clamp_index(_IQint(_IQmpy(particlePos[XID(i)], invertSpacing)), 0, (int)CellNumX - 1);
        unsigned int yi = (unsigned int)clamp_index(_IQint(_IQmpy(particlePos[YID(i)], invertSpacing)), 0, (int)CellNumY - 1);
        unsigned int cellNr = INDEX(xi, yi);
        unsigned int gridIndex = --cellParticleCountPrefix[cellNr];
        particlePosId[gridIndex] = i;
    }

    const _iq minDist = _IQmpy(_IQ(2.0), ParticleRadius);
    const _iq minDist2 = _IQmpy(minDist, minDist);

    for (unsigned int iter = 0; iter < nIters; iter++) {
        for (unsigned int i = 0; i < activeParticles; i++) {
            _iq px = particlePos[XID(i)];
            _iq py = particlePos[YID(i)];

            int pxi = clamp_index(_IQint(_IQmpy(px, invertSpacing)), 0, (int)CellNumX - 1);
            int pyi = clamp_index(_IQint(_IQmpy(py, invertSpacing)), 0, (int)CellNumY - 1);
            int x0 = (pxi > 0) ? pxi - 1 : 0;
            int y0 = (pyi > 0) ? pyi - 1 : 0;
            int x1 = (pxi + 1 < (int)CellNumX) ? pxi + 1 : (int)CellNumX - 1;
            int y1 = (pyi + 1 < (int)CellNumY) ? pyi + 1 : (int)CellNumY - 1;

            for (int xi = x0; xi <= x1; xi++) {
                for (int yi = y0; yi <= y1; yi++) {
                    unsigned int cellNr = INDEX((unsigned int)xi, (unsigned int)yi);
                    unsigned int firstIdx = cellParticleCountPrefix[cellNr];
                    unsigned int lastIdx = cellParticleCountPrefix[cellNr + 1U];
                    for (unsigned int j = firstIdx; j < lastIdx; j++) {
                        unsigned int id = particlePosId[j];
                        if (id == i) {
                            continue;
                        }

                        _iq qx = particlePos[XID(id)];
                        _iq qy = particlePos[YID(id)];
                        _iq dx = qx - px;
                        _iq dy = qy - py;
                        _iq d2 = _IQmpy(dx, dx) + _IQmpy(dy, dy);
                        if (d2 > minDist2 || d2 == _IQ(0.0)) {
                            continue;
                        }

                        _iq d = _IQsqrt(d2);
                        _iq s = _IQdiv(_IQmpy(_IQ(0.5), (minDist - d)), d);
                        dx = _IQmpy(dx, s);
                        dy = _IQmpy(dy, s);
                        particlePos[XID(i)] -= dx;
                        particlePos[YID(i)] -= dy;
                        particlePos[XID(id)] += dx;
                        particlePos[YID(id)] += dy;
                    }
                }
            }
        }
    }
    for (unsigned int i = 0; i < activeParticles; i++) {
        _iq x = particlePos[XID(i)];
        _iq y = particlePos[YID(i)];

        if (x < minX) {
            x = minX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (x > maxX) {
            x = maxX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (y < minY) {
            y = minY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)] ,BOUNCYNESS);
        }
        if (y > maxY) {
            y = maxY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)] ,BOUNCYNESS);
        }

        particlePos[XID(i)] = x;
        particlePos[YID(i)] = y;
    }

    ConstrainParticlesToFluid();
}

void density_update(void) {
    memset(particleDensity, 0, sizeof(particleDensity));

    const _iq h = Spacing;
    const _iq h1 = invertSpacing;
    const _iq h2 = _IQmpy(_IQ(0.5), h);

    for (unsigned int i = 0; i < activeParticles; i++) {
        _iq x = clampf_local(particlePos[XID(i)], h, _IQmpy(_IQ(CellNumX - 1U), h));
        _iq y = clampf_local(particlePos[YID(i)], h, _IQmpy(_IQ(CellNumY - 1U), h));

        int x0 = _IQint(_IQmpy((x - h2), h1));
        int y0 = _IQint(_IQmpy((y - h2), h1));
        x0 = clamp_index(x0, 0, (int)CellNumX - 2);
        y0 = clamp_index(y0, 0, (int)CellNumY - 2);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        _iq tx = _IQmpy(((x - h2) - _IQmpy(_IQ(x0), h)), h1);
        _iq ty = _IQmpy(((y - h2) - _IQmpy(_IQ(y0), h)), h1);
        _iq sx = _IQ(1.0) - tx;
        _iq sy = _IQ(1.0) - ty;

        particleDensity[INDEX(x0, y0)] += _IQmpy(sx, sy);
        particleDensity[INDEX(x1, y0)] += _IQmpy(tx, sy);
        particleDensity[INDEX(x1, y1)] += _IQmpy(tx, ty);
        particleDensity[INDEX(x0, y1)] += _IQmpy(sx, ty);
    }

    if (particleRestDensity == _IQ(0.0)) {
        _iq sum = _IQ(0.0);
        unsigned int numFluid = 0;
        for (unsigned int i = 0; i < CellCount; i++) {
            if (cellType[i] == FLUID_CELL) {
                sum += particleDensity[i];
                numFluid++;
            }
        }
        if (numFluid > 0U) {
            particleRestDensity = _IQdiv(sum, _IQ(numFluid));
        }
    }
}

void particles_to_grid(void) {
    //memcpy(uPrev, uVel, sizeof(uVel));
    //memcpy(vPrev, vVel, sizeof(vVel));

    memset(uVel, 0, sizeof(uVel));
    memset(vVel, 0, sizeof(vVel));
    memset(uWeight, 0, sizeof(uWeight));
    memset(vWeight, 0, sizeof(vWeight));

    for (unsigned int i = 0; i < CellCount; i++) {
        cellType[i] = (solidMask[i] == _IQ(0.0)) ? SOLID_CELL : AIR_CELL;
    }

    for (unsigned int i = 0; i < activeParticles; i++) {
        int xi = clamp_index(_IQint(_IQmpy(particlePos[XID(i)], invertSpacing)), 0, (int)CellNumX - 1);
        int yi = clamp_index(_IQint(_IQmpy(particlePos[YID(i)], invertSpacing)), 0, (int)CellNumY - 1);
        unsigned int cellNr = INDEX((unsigned int)xi, (unsigned int)yi);
        cellType[cellNr] = FLUID_CELL;
        
    }

    const _iq h = Spacing;
    const _iq h1 = invertSpacing;
    const _iq h2 = _IQmpy(_IQ(0.5), h);

    for (int component = 0; component < 2; component++) {
        _iq dx = (component == 0) ? _IQ(0.0) : h2;
        _iq dy = (component == 0) ? h2 : _IQ(0.0);
        _iq *f = (component == 0) ? uVel : vVel;
        _iq *w = (component == 0) ? uWeight : vWeight;

        for (unsigned int i = 0; i < activeParticles; i++) {
            _iq x = clampf_local(particlePos[XID(i)], h, _IQmpy(_IQ(CellNumX - 1U), h));
            _iq y = clampf_local(particlePos[YID(i)], h, _IQmpy(_IQ(CellNumY - 1U), h));

            int x0 = _IQint(_IQmpy((x - dx), h1));
            int y0 = _IQint(_IQmpy((y - dy), h1));
            x0 = clamp_index(x0, 0, (int)CellNumX - 2);
            y0 = clamp_index(y0, 0, (int)CellNumY - 2);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            _iq tx = _IQmpy(((x - dx) - _IQmpy(_IQ(x0), h)), h1);
            _iq ty = _IQmpy(((y - dy) - _IQmpy(_IQ(y0), h)), h1);
            _iq sx = _IQ(1.0) - tx;
            _iq sy = _IQ(1.0) - ty;

            _iq w0 = _IQmpy(sx, sy);
            _iq w1 = _IQmpy(tx, sy);
            _iq w2 = _IQmpy(tx, ty);
            _iq w3 = _IQmpy(sx, ty);

            _iq pv = particleVel[2 * i + (unsigned int)component];
            unsigned int nr0 = INDEX((unsigned int)x0, (unsigned int)y0);
            unsigned int nr1 = INDEX((unsigned int)x1, (unsigned int)y0);
            unsigned int nr2 = INDEX((unsigned int)x1, (unsigned int)y1);
            unsigned int nr3 = INDEX((unsigned int)x0, (unsigned int)y1);

            f[nr0] += _IQmpy(pv, w0); w[nr0] += w0;
            f[nr1] += _IQmpy(pv, w1); w[nr1] += w1;
            f[nr2] += _IQmpy(pv, w2); w[nr2] += w2;
            f[nr3] += _IQmpy(pv, w3); w[nr3] += w3;
        }

        for (unsigned int i = 0; i < CellCount; i++) {
            if (w[i] > _IQ(0.0)) {
                f[i] = _IQdiv(f[i], w[i]);
            }
        }

        for (unsigned int x = 0; x < CellNumX; x++) {
            for (unsigned int y = 0; y < CellNumY; y++) {
                unsigned int idx = INDEX(x, y);
                unsigned int solid = (cellType[idx] == SOLID_CELL) ? 1U : 0U;
                if (component == 0) {
                    unsigned int leftSolid = (x > 0U && cellType[INDEX(x - 1U, y)] == SOLID_CELL) ? 1U : 0U;
                    if (solid || leftSolid) {
                        uVel[idx] = uPrev[idx];
                    }
                } else {
                    unsigned int bottomSolid = (y > 0U && cellType[INDEX(x, y - 1U)] == SOLID_CELL) ? 1U : 0U;
                    if (solid || bottomSolid) {
                        vVel[idx] = vPrev[idx];
                    }
                }
            }
        }
    }
}

void compute_grid_forces(unsigned int nIters) {
    //memset(pressure, 0, sizeof(pressure));
    memcpy(uPrev, uVel, sizeof(uVel));
    memcpy(vPrev, vVel, sizeof(vVel));



    //const _iq cp = _IQdiv(_IQmpy(_IQ(1000.0), Spacing), dt);

    for (unsigned int iter = 0; iter < nIters; iter++) {
        for (unsigned int x = 1; x < CellNumX - 1U; x++) {
            for (unsigned int y = 1; y < CellNumY - 1U; y++) {
                unsigned int center = INDEX(x, y);
                if (cellType[center] != FLUID_CELL) {
                    continue;
                }

                unsigned int left = INDEX(x - 1U, y);
                unsigned int right = INDEX(x + 1U, y);
                unsigned int bottom = INDEX(x, y - 1U);
                unsigned int top = INDEX(x, y + 1U);

                _iq sx0 = solidMask[left];
                _iq sx1 = solidMask[right];
                _iq sy0 = solidMask[bottom];
                _iq sy1 = solidMask[top];
                _iq s = sx0 + sx1 + sy0 + sy1;
                if (s == _IQ(0.0)) {
                    continue;
                }

                _iq div = uVel[right] - uVel[center] + vVel[top] - vVel[center];

                if (particleRestDensity > _IQ(0.0)) {
                    _iq compression = particleDensity[center] - particleRestDensity;
                    if (compression > _IQ(0.0)) {
                        compression = _IQmpy(compression,stiffnessCoefficient);
                        div -= compression;
                    }
                }

                _iq p = -_IQdiv(div, s);
                p = _IQmpy(p, overRelaxiation);
                //pressure[center] += _IQmpy(cp, p);

                uVel[center] -= _IQmpy(sx0, p);
                uVel[right] += _IQmpy(sx1, p);
                vVel[center] -= _IQmpy(sy0, p);
                vVel[top] += _IQmpy(sy1, p);
            }
        }
    }
}

void grid_to_particles(void) {
    const _iq flipRatio = _IQ(0.9);
    const _iq h = Spacing;
    const _iq h1 = invertSpacing;
    const _iq h2 = _IQmpy(_IQ(0.5), h);

    for (int component = 0; component < 2; component++) {
        _iq dx = (component == 0) ? _IQ(0.0) : h2;
        _iq dy = (component == 0) ? h2 : _IQ(0.0);
        _iq *f = (component == 0) ? uVel : vVel;
        _iq *prevF = (component == 0) ? uPrev : vPrev;
        int offset = (component == 0) ? (int)CellNumY : 1;

        for (unsigned int i = 0; i < activeParticles; i++) {
            _iq x = clampf_local(particlePos[XID(i)], h, _IQmpy(_IQ(CellNumX - 1U), h));
            _iq y = clampf_local(particlePos[YID(i)], h, _IQmpy(_IQ(CellNumY - 1U), h));

            int x0 = clamp_index(_IQint(_IQmpy((x - dx), h1)), 0, (int)CellNumX - 2);
            int y0 = clamp_index(_IQint(_IQmpy((y - dy), h1)), 0, (int)CellNumY - 2);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            _iq tx = _IQmpy(((x - dx) - _IQmpy(_IQ(x0), h)), h1);
            _iq ty = _IQmpy(((y - dy) - _IQmpy(_IQ(y0), h)), h1);
            _iq sx = _IQ(1.0) - tx;
            _iq sy = _IQ(1.0) - ty;

            _iq d0 = _IQmpy(sx, sy);
            _iq d1 = _IQmpy(tx, sy);
            _iq d2 = _IQmpy(tx, ty);
            _iq d3 = _IQmpy(sx, ty);

            unsigned int nr0 = INDEX((unsigned int)x0, (unsigned int)y0);
            unsigned int nr1 = INDEX((unsigned int)x1, (unsigned int)y0);
            unsigned int nr2 = INDEX((unsigned int)x1, (unsigned int)y1);
            unsigned int nr3 = INDEX((unsigned int)x0, (unsigned int)y1);

            _iq valid0 = (cellType[nr0] != AIR_CELL || ((int)nr0 - offset >= 0 && cellType[(unsigned int)((int)nr0 - offset)] != AIR_CELL)) ? _IQ(1.0) : _IQ(0.0);
            _iq valid1 = (cellType[nr1] != AIR_CELL || ((int)nr1 - offset >= 0 && cellType[(unsigned int)((int)nr1 - offset)] != AIR_CELL)) ? _IQ(1.0) : _IQ(0.0);
            _iq valid2 = (cellType[nr2] != AIR_CELL || ((int)nr2 - offset >= 0 && cellType[(unsigned int)((int)nr2 - offset)] != AIR_CELL)) ? _IQ(1.0) : _IQ(0.0);
            _iq valid3 = (cellType[nr3] != AIR_CELL || ((int)nr3 - offset >= 0 && cellType[(unsigned int)((int)nr3 - offset)] != AIR_CELL)) ? _IQ(1.0) : _IQ(0.0);

            _iq d = _IQmpy(valid0, d0) + _IQmpy(valid1, d1) + _IQmpy(valid2, d2) + _IQmpy(valid3, d3);
            if (d <= _IQ(0.0)) {
                continue;
            }

            _iq picV = _IQdiv(_IQmpy(valid0, _IQmpy(d0, f[nr0])) + _IQmpy(valid1, _IQmpy(d1, f[nr1])) + _IQmpy(valid2, _IQmpy(d2, f[nr2])) + _IQmpy(valid3, _IQmpy(d3, f[nr3])), d);
            _iq corr = _IQdiv(_IQmpy(valid0, _IQmpy(d0, (f[nr0] - prevF[nr0]))) + _IQmpy(valid1, _IQmpy(d1, (f[nr1] - prevF[nr1])))
                         + _IQmpy(valid2, _IQmpy(d2, (f[nr2] - prevF[nr2]))) + _IQmpy(valid3, _IQmpy(d3, (f[nr3] - prevF[nr3]))), d);
            _iq oldV = particleVel[2 * i + (unsigned int)component];
            _iq flipV = oldV + corr;
            particleVel[2 * i + (unsigned int)component] = _IQmpy((_IQ(1.0) - flipRatio), picV) + _IQmpy(flipRatio, flipV);
        }
    }
}

void screen_update() {
    // 显示：有粒子的网格点亮；墙壁只点亮心形蓝色轮廓，背景保持熄灭
    for (int y = 1; y < CellNumY-1; y++) {
        for (int x = 1; x < CellNumX-1; x++) {
            uint8_t cell = cellType[INDEX(x, y)];
            uint8_t on = 0U;
            if (cell == FLUID_CELL) {
                on = 1U;
            } else if (cell == SOLID_CELL && is_visible_wall((unsigned int)x, (unsigned int)y)) {
                on = 1U;
            }
            LED_SetPixel((uint16_t)((x-1)*16 + (y-1)), on ? LEDON : LEDOFF);
        }
    }
}

