#include "goal_distribute/game_state.hpp"
#include <algorithm>

namespace goal_distribute {

GameState::GameState()
    : self_hp_(MAX_HP)
    , self_ammo_(MAX_AMMO)
    , enemy_visible_(false)
    , enemy_x_(0.0)
    , enemy_y_(0.0)
    , enemy_z_(0.0)
    , game_time_(420)  // 默认7分钟
    , team_color_("red")
{
}

void GameState::setSelfHP(uint16_t hp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    self_hp_ = std::min(hp, MAX_HP);
    if (state_update_callback_) {
        state_update_callback_();
    }
}

void GameState::setSelfAmmo(uint16_t ammo) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    self_ammo_ = std::min(ammo, MAX_AMMO);
    if (state_update_callback_) {
        state_update_callback_();
    }
}

uint16_t GameState::getSelfHP() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return self_hp_;
}

uint16_t GameState::getSelfAmmo() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return self_ammo_;
}

double GameState::getSelfHPPercent() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return static_cast<double>(self_hp_) / MAX_HP;
}

double GameState::getSelfAmmoPercent() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return static_cast<double>(self_ammo_) / MAX_AMMO;
}

void GameState::setBuildingHP(BuildingType type, uint16_t hp, bool is_ally) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    building_hp_[std::make_pair(is_ally, type)] = hp;
    if (state_update_callback_) {
        state_update_callback_();
    }
}

uint16_t GameState::getBuildingHP(BuildingType type, bool is_ally) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto key = std::make_pair(is_ally, type);
    auto it = building_hp_.find(key);
    if (it != building_hp_.end()) {
        return it->second;
    }
    return 0;
}

double GameState::getBuildingHPPercent(BuildingType type, bool is_ally) const {
    uint16_t hp = getBuildingHP(type, is_ally);
    // 假设最大血量为600（根据实际比赛规则调整）
    constexpr uint16_t MAX_BUILDING_HP = 600;
    return static_cast<double>(hp) / MAX_BUILDING_HP;
}

void GameState::setEnemyVisible(bool visible) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enemy_visible_ = visible;
}

void GameState::setEnemyPosition(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enemy_x_ = x;
    enemy_y_ = y;
    enemy_z_ = z;
    enemy_visible_ = true;
}

bool GameState::isEnemyVisible() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return enemy_visible_;
}

void GameState::getEnemyPosition(double& x, double& y, double& z) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    x = enemy_x_;
    y = enemy_y_;
    z = enemy_z_;
}

void GameState::setGameTime(uint32_t remaining_seconds) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    game_time_ = remaining_seconds;
}

void GameState::setTeamColor(const std::string& color) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    team_color_ = color;
}

uint32_t GameState::getGameTime() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return game_time_;
}

std::string GameState::getTeamColor() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return team_color_;
}

bool GameState::isSelfLowHP() const {
    return getSelfHPPercent() < LOW_HP_THRESHOLD;
}

bool GameState::isSelfLowAmmo() const {
    return getSelfAmmoPercent() < LOW_AMMO_THRESHOLD;
}

bool GameState::isBaseInDanger() const {
    double base_hp_percent = getBuildingHPPercent(BuildingType::BASE, true);
    return base_hp_percent < BUILDING_DANGER_THRESHOLD;
}

bool GameState::isOutpostInDanger() const {
    double outpost_hp_percent = getBuildingHPPercent(BuildingType::SENTRY_OUTPOST, true);
    return outpost_hp_percent < BUILDING_DANGER_THRESHOLD;
}

bool GameState::needRetreat() const {
    // 需要撤退的情况：
    // 1. 自身血量过低
    // 2. 基地或前哨站处于危险状态
    return isSelfLowHP() || isBaseInDanger() || isOutpostInDanger();
}

void GameState::registerStateUpdateCallback(StateUpdateCallback callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_update_callback_ = callback;
}

void GameState::setAllBuildingHP(const std::map<BuildingType, uint16_t>& ally_buildings,
                                 const std::map<BuildingType, uint16_t>& enemy_buildings) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 清空现有建筑血量
    building_hp_.clear();
    
    // 设置友方建筑血量
    for (const auto& pair : ally_buildings) {
        building_hp_[std::make_pair(true, pair.first)] = pair.second;
    }
    
    // 设置敌方建筑血量
    for (const auto& pair : enemy_buildings) {
        building_hp_[std::make_pair(false, pair.first)] = pair.second;
    }
    
    if (state_update_callback_) {
        state_update_callback_();
    }
}

std::map<std::pair<bool, GameState::BuildingType>, uint16_t> GameState::getAllBuildingHP() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return building_hp_;
}

void GameState::updateAllState(uint16_t self_hp, uint16_t self_ammo,
                              const std::map<BuildingType, uint16_t>& ally_buildings,
                              const std::map<BuildingType, uint16_t>& enemy_buildings,
                              bool enemy_visible, double enemy_x, 
                              double enemy_y, double enemy_z) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    self_hp_ = std::min(self_hp, MAX_HP);
    self_ammo_ = std::min(self_ammo, MAX_AMMO);
    
    setAllBuildingHP(ally_buildings, enemy_buildings);
    
    enemy_visible_ = enemy_visible;
    if (enemy_visible) {
        enemy_x_ = enemy_x;
        enemy_y_ = enemy_y;
        enemy_z_ = enemy_z;
    }
    
    if (state_update_callback_) {
        state_update_callback_();
    }
}

void GameState::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    self_hp_ = MAX_HP;
    self_ammo_ = MAX_AMMO;
    building_hp_.clear();
    enemy_visible_ = false;
    enemy_x_ = enemy_y_ = enemy_z_ = 0.0;
    game_time_ = 420;
}

} // namespace goal_distribute
