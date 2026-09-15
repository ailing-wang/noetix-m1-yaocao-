#ifndef KEYBUFFER_H
#define KEYBUFFER_H

enum class ControlIndex
{
    // 按钮索引 (0-10)
    BUTTON_A = 0,
    BUTTON_B,
    BUTTON_X,
    BUTTON_Y,
    BUTTON_LB,
    BUTTON_RB,
    BUTTON_START,
    BUTTON_BACK,
    BUTTON_HOME,
    BUTTON_LO,
    BUTTON_RO,

    // 摇杆索引 (11-14)
    STICK_LX,
    STICK_LY,
    STICK_RX,
    STICK_RY,

    // 扳机/方向键索引 (15-20)
    TRIGGER_LT,
    TRIGGER_RT,
    TRIGGER_XX,
    TRIGGER_YY,

    // 总数量（用于边界检查）
    CONTROL_COUNT
};

struct KeyBuffer
{
    struct Buttons
    {
        int a, b, x, y; // 主要按钮
        int lb, rb;
        int start, back, home;
        int lo, ro;
    } buttons;

    struct Sticks
    {
        int lx, ly;
        int rx, ry;
    } sticks;

    struct Triggers
    {
        int lt, rt;
        int xx, yy;
    } triggers;

    // 构造函数
    KeyBuffer()
    {
        resetAll();
    }

    void resetAll()
    {
        // 重置数字按钮
        buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        // 重置摇杆值
        sticks = {0, 0, 0, 0};

        // 重置扳机和方向键
        triggers = {-32767, -32767, 0, 0};
    }

    int getValueByIndex(ControlIndex index) const
    {
        switch (index)
        {
        // 按钮
        case ControlIndex::BUTTON_A:
            return buttons.a;
        case ControlIndex::BUTTON_B:
            return buttons.b;
        case ControlIndex::BUTTON_X:
            return buttons.x;
        case ControlIndex::BUTTON_Y:
            return buttons.y;
        case ControlIndex::BUTTON_LB:
            return buttons.lb;
        case ControlIndex::BUTTON_RB:
            return buttons.rb;
        case ControlIndex::BUTTON_START:
            return buttons.start;
        case ControlIndex::BUTTON_BACK:
            return buttons.back;
        case ControlIndex::BUTTON_HOME:
            return buttons.home;
        case ControlIndex::BUTTON_LO:
            return buttons.lo;
        case ControlIndex::BUTTON_RO:
            return buttons.ro;

        // 摇杆
        case ControlIndex::STICK_LX:
            return sticks.lx;
        case ControlIndex::STICK_LY:
            return sticks.ly;
        case ControlIndex::STICK_RX:
            return sticks.rx;
        case ControlIndex::STICK_RY:
            return sticks.ry;

        // 扳机/方向键
        case ControlIndex::TRIGGER_LT:
            return triggers.lt;
        case ControlIndex::TRIGGER_RT:
            return triggers.rt;
        case ControlIndex::TRIGGER_XX:
            return triggers.xx;
        case ControlIndex::TRIGGER_YY:
            return triggers.yy;

        default:
            return 0;
        }
    }

    // 便捷函数：检查按钮是否按下
    bool isButtonPressed(ControlIndex index) const
    {
        return getValueByIndex(index) == 1;
    }
};
#endif