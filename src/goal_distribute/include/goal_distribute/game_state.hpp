#ifndef GOAL_DISTRIBUTE_GAME_STATE_HPP
#define GOAL_DISTRIBUTE_GAME_STATE_HPP

#include <ros/ros.h>
#include <cstdint>
#include <string>
#include <map>
#include <mutex>
#include <functional>

namespace goal_distribute {

/**
 * @brief RoboMaster游戏状态管理类
 * 管理自身血量、弹丸数量、建筑血量等游戏状态信息
 */
class GameState {
public:
    // 建筑类型枚举
    enum class BuildingType {
        SENTRY_OUTPOST,     // 哨兵前哨站
        BASE,               // 基地
        ENGINEER_STATION    // 工程站
    };

    // 状态更新回调函数类型
    using StateUpdateCallback = std::function<void()>;

    GameState();
    ~GameState() = default;

    // ========== 自身状态 ==========
    void setSelfHP(uint16_t hp);
    void setSelfAmmo(uint16_t ammo);
    uint16_t getSelfHP() const;
    uint16_t getSelfAmmo() const;
    double getSelfHPPercent() const;
    double getSelfAmmoPercent() const;

    // ========== 建筑血量管理 ==========
    void setBuildingHP(BuildingType type, uint16_t hp, bool is_ally = true);
    uint16_t getBuildingHP(BuildingType type, bool is_ally = true) const;
    double getBuildingHPPercent(BuildingType type, bool is_ally = true) const;
    
    // 批量设置建筑血量（用于一次性更新所有建筑状态）
    void setAllBuildingHP(const std::map<BuildingType, uint16_t>& ally_buildings,
                          const std::map<BuildingType, uint16_t>& enemy_buildings);
    
    // 获取所有建筑血量
    std::map<std::pair<bool, BuildingType>, uint16_t> getAllBuildingHP() const;

    // ========== 敌人状态 ==========
    void setEnemyVisible(bool visible);
    void setEnemyPosition(double x, double y, double z);
    bool isEnemyVisible() const;
    void getEnemyPosition(double& x, double& y, double& z) const;

    // ========== 比赛状态 ==========
    void setGameTime(uint32_t remaining_seconds);
    void setTeamColor(const std::string& color);  // "red" or "blue"
    uint32_t getGameTime() const;
    std::string getTeamColor() const;
    
    // ========== 批量状态更新接口 ==========
    /**
     * @brief 批量更新所有状态（用于从统一的状态消息中更新）
     * 这是一个便捷接口，用于一次性更新多个状态
     */
    void updateAllState(uint16_t self_hp, uint16_t self_ammo,
                       const std::map<BuildingType, uint16_t>& ally_buildings,
                       const std::map<BuildingType, uint16_t>& enemy_buildings,
                       bool enemy_visible = false, double enemy_x = 0, 
                       double enemy_y = 0, double enemy_z = 0);

    // ========== 高级状态判断 ==========
    bool isSelfLowHP() const;           // 自身血量是否过低
    bool isSelfLowAmmo() const;         // 弹丸是否不足
    bool isBaseInDanger() const;        // 基地是否危险
    bool isOutpostInDanger() const;     // 前哨站是否危险
    bool needRetreat() const;           // 是否需要撤退

    // ========== 注册状态更新回调 ==========
    void registerStateUpdateCallback(StateUpdateCallback callback);

    // ========== 重置和清理 ==========
    void reset();

private:
    // 最大血量（根据比赛规则设置）
    static constexpr uint16_t MAX_HP = 600;
    static constexpr uint16_t MAX_AMMO = 500;
    
    // 阈值常量
    static constexpr double LOW_HP_THRESHOLD = 0.3;      // 30%
    static constexpr double LOW_AMMO_THRESHOLD = 0.2;   // 20%
    static constexpr double BUILDING_DANGER_THRESHOLD = 0.5;  // 50%

    mutable std::mutex state_mutex_;

    // 自身状态
    uint16_t self_hp_;
    uint16_t self_ammo_;

    // 建筑血量 (is_ally -> building_type -> hp)
    std::map<std::pair<bool, BuildingType>, uint16_t> building_hp_;

    // 敌人状态
    bool enemy_visible_;
    double enemy_x_, enemy_y_, enemy_z_;

    // 比赛状态
    uint32_t game_time_;
    std::string team_color_;

    // 回调函数
    StateUpdateCallback state_update_callback_;
};

} // namespace goal_distribute

#endif // GOAL_DISTRIBUTE_GAME_STATE_HPP
