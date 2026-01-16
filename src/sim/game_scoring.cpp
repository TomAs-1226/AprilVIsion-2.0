/**
 * @file game_scoring.cpp
 * @brief FRC game scoring implementation
 */

#include "game_scoring.hpp"
#include <cmath>
#include <algorithm>

namespace frc_vision {
namespace sim {

GameScoringSystem::GameScoringSystem() {
    setup_rebuilt_2026(); // Default
}

void GameScoringSystem::initialize(GameYear year, Alliance alliance) {
    state_.game_year = year;
    state_.alliance = alliance;

    switch (year) {
        case GameYear::CRESCENDO_2024:
            setup_crescendo_2024();
            break;
        case GameYear::REEFSCAPE_2025:
            setup_reefscape_2025();
            break;
        case GameYear::REBUILT_2026:
        default:
            setup_rebuilt_2026();
            break;
    }

    reset_match();
}

void GameScoringSystem::setup_crescendo_2024() {
    current_game_.year = GameYear::CRESCENDO_2024;
    current_game_.name = "CRESCENDO";
    current_game_.field_layout_file = "2024-crescendo.json";
    current_game_.auto_duration = 15.0;
    current_game_.teleop_duration = 135.0;
    current_game_.endgame_start = 130.0;

    current_game_.game_piece_types = {"note"};

    // Auto actions
    current_game_.auto_actions = {
        {"Leave Starting Zone", "leave_zone", 2, {}, 0, false},
        {"Score Speaker", "score_speaker_auto", 5, {3, 4, 7, 8}, 2.0, true},
        {"Score Amp", "score_amp_auto", 2, {5, 6, 15, 16}, 1.5, true}
    };

    // Teleop actions
    current_game_.teleop_actions = {
        {"Score Speaker", "score_speaker", 2, {3, 4, 7, 8}, 2.0, true},
        {"Score Amp", "score_amp", 1, {5, 6, 15, 16}, 1.5, true},
        {"Amplify", "amplify", 0, {5, 6, 15, 16}, 1.5, false}
    };

    // Endgame actions
    current_game_.endgame_actions = {
        {"Park", "park", 1, {11, 12, 13, 14, 15, 16}, 1.0, false},
        {"Stage Climb", "climb_stage", 3, {11, 12, 13, 14, 15, 16}, 1.0, false},
        {"Spotlight", "spotlight", 1, {11, 12, 13, 14, 15, 16}, 1.0, false},
        {"Trap", "trap", 5, {11, 12, 13, 14, 15, 16}, 1.0, true}
    };

    // Starting positions
    current_game_.start_positions_blue = {
        {1.35, 5.55, 0.0},
        {1.35, 4.10, 0.0},
        {1.35, 2.65, 0.0}
    };
    current_game_.start_positions_red = {
        {15.19, 5.55, M_PI},
        {15.19, 4.10, M_PI},
        {15.19, 2.65, M_PI}
    };
}

void GameScoringSystem::setup_reefscape_2025() {
    current_game_.year = GameYear::REEFSCAPE_2025;
    current_game_.name = "REEFSCAPE";
    current_game_.field_layout_file = "2025-reefscape.json";
    current_game_.auto_duration = 15.0;
    current_game_.teleop_duration = 135.0;
    current_game_.endgame_start = 130.0;

    current_game_.game_piece_types = {"coral", "algae"};

    // Auto actions
    current_game_.auto_actions = {
        {"Leave Starting Zone", "leave_zone", 3, {}, 0, false},
        {"Score Coral L1", "score_coral_l1_auto", 3, {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, 1.5, true},
        {"Score Coral L2", "score_coral_l2_auto", 4, {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, 1.5, true},
        {"Score Coral L3", "score_coral_l3_auto", 6, {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, 1.5, true},
        {"Score Coral L4", "score_coral_l4_auto", 7, {4, 5, 14, 15}, 1.5, true}
    };

    // Teleop actions
    current_game_.teleop_actions = {
        {"Score Coral L1", "score_coral_l1", 2, {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, 1.5, true},
        {"Score Coral L2", "score_coral_l2", 4, {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, 1.5, true},
        {"Score Coral L3", "score_coral_l3", 6, {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, 1.5, true},
        {"Score Coral L4", "score_coral_l4", 7, {4, 5, 14, 15}, 1.5, true},
        {"Score Algae Processor", "score_algae_processor", 6, {3, 16}, 1.5, true},
        {"Score Algae Net", "score_algae_net", 4, {}, 5.0, true}
    };

    // Endgame actions
    current_game_.endgame_actions = {
        {"Shallow Cage", "cage_shallow", 2, {4, 5, 14, 15}, 2.0, false},
        {"Deep Cage", "cage_deep", 6, {4, 5, 14, 15}, 2.0, false},
        {"Barge Park", "barge_park", 12, {}, 5.0, false}
    };

    // Starting positions (near coral stations)
    current_game_.start_positions_blue = {
        {1.5, 1.0, 0.0},
        {1.5, 4.0, 0.0},
        {1.5, 7.0, 0.0}
    };
    current_game_.start_positions_red = {
        {16.0, 1.0, M_PI},
        {16.0, 4.0, M_PI},
        {16.0, 7.0, M_PI}
    };
}

void GameScoringSystem::setup_rebuilt_2026() {
    current_game_.year = GameYear::REBUILT_2026;
    current_game_.name = "REBUILT";
    current_game_.field_layout_file = "2026-rebuilt.json";
    current_game_.auto_duration = 15.0;
    current_game_.teleop_duration = 135.0;
    current_game_.endgame_start = 130.0;

    current_game_.game_piece_types = {"artifact", "fuel"};

    // Auto actions
    current_game_.auto_actions = {
        {"Leave Starting Zone", "leave_zone", 3, {}, 0, false},
        {"Cross Obstacle", "cross_obstacle", 4, {}, 0, false},
        {"Score Fuel Auto", "score_fuel_auto", 2, {13, 14, 15, 16, 29, 30, 31, 32}, 1.5, true},
        {"Excavate Artifact", "excavate_artifact", 2, {1, 6, 7, 12, 17, 22, 23, 28}, 1.5, false}
    };

    // Teleop actions
    current_game_.teleop_actions = {
        {"Score Fuel", "score_fuel", 1, {13, 14, 15, 16, 29, 30, 31, 32}, 1.5, true},
        {"Artifact Low", "artifact_low", 2, {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}, 1.5, true},
        {"Artifact Mid", "artifact_mid", 4, {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}, 1.5, true},
        {"Artifact High", "artifact_high", 6, {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}, 1.5, true},
        {"Cross Obstacle", "cross_obstacle", 2, {}, 0, false}
    };

    // Endgame actions
    current_game_.endgame_actions = {
        {"Tower Climb Low", "climb_low", 4, {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}, 2.0, false},
        {"Tower Climb Mid", "climb_mid", 8, {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}, 2.0, false},
        {"Tower Climb High", "climb_high", 12, {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}, 2.0, false}
    };

    // Starting positions
    current_game_.start_positions_blue = {
        {1.5, 1.5, 0.0},
        {1.5, 4.0, 0.0},
        {1.5, 6.5, 0.0}
    };
    current_game_.start_positions_red = {
        {15.0, 1.5, M_PI},
        {15.0, 4.0, M_PI},
        {15.0, 6.5, M_PI}
    };
}

void GameScoringSystem::update(double dt) {
    state_.match_time += dt;

    // Update match phase
    state_.is_auto = (state_.match_time < current_game_.auto_duration);
    state_.is_endgame = (state_.match_time >= current_game_.endgame_start);
}

void GameScoringSystem::reset_match() {
    state_.match_time = 0;
    state_.is_auto = true;
    state_.is_endgame = false;
    state_.auto_score = 0;
    state_.teleop_score = 0;
    state_.endgame_score = 0;
    state_.held_pieces.clear();
    state_.pieces_scored = 0;
    state_.completed_objectives.clear();
}

int GameScoringSystem::try_score(const Pose2D& pose, const std::string& action_name) {
    // Get appropriate action list
    const std::vector<ScoringAction>* actions = nullptr;
    int* score_ptr = nullptr;

    if (state_.is_endgame) {
        actions = &current_game_.endgame_actions;
        score_ptr = &state_.endgame_score;
    } else if (state_.is_auto) {
        actions = &current_game_.auto_actions;
        score_ptr = &state_.auto_score;
    } else {
        actions = &current_game_.teleop_actions;
        score_ptr = &state_.teleop_score;
    }

    // Find action
    for (const auto& action : *actions) {
        if (action.code_name == action_name) {
            // Check requirements
            if (action.requires_game_piece && state_.held_pieces.empty()) {
                return 0; // Need game piece
            }

            // Check proximity if tags specified
            if (!action.valid_tags.empty()) {
                bool near_valid_tag = false;
                for (int tag_id : action.valid_tags) {
                    if (is_near_scoring_location(pose, tag_id, action.proximity_radius)) {
                        near_valid_tag = true;
                        break;
                    }
                }
                if (!near_valid_tag) {
                    return 0;
                }
            }

            // Score!
            *score_ptr += action.points;
            state_.pieces_scored++;

            // Remove game piece if required
            if (action.requires_game_piece && !state_.held_pieces.empty()) {
                state_.held_pieces.pop_back();
            }

            // Track objective
            state_.completed_objectives.push_back(action_name);

            return action.points;
        }
    }

    return 0;
}

bool GameScoringSystem::is_near_scoring_location(const Pose2D& pose, int tag_id, double radius) const {
    if (!field_.has_tag(tag_id)) {
        return false;
    }

    const auto& tag = field_.get_tag(tag_id);
    double dx = tag.center_field.x - pose.x;
    double dy = tag.center_field.y - pose.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    return dist <= radius;
}

bool GameScoringSystem::pickup_game_piece(const std::string& type) {
    // Check if valid type for current game
    bool valid_type = false;
    for (const auto& t : current_game_.game_piece_types) {
        if (t == type) {
            valid_type = true;
            break;
        }
    }

    if (!valid_type) {
        return false;
    }

    // Max 1 piece for most games (could be configurable)
    if (state_.held_pieces.size() >= 1) {
        return false;
    }

    GamePiece piece;
    piece.type = type;
    piece.held = true;
    piece.pickup_time = state_.match_time;
    state_.held_pieces.push_back(piece);

    return true;
}

std::vector<ScoringAction> GameScoringSystem::get_available_actions(
    const Pose2D& pose, const std::vector<int>& visible_tags) const {

    std::vector<ScoringAction> available;

    const std::vector<ScoringAction>* actions = nullptr;
    if (state_.is_endgame) {
        actions = &current_game_.endgame_actions;
    } else if (state_.is_auto) {
        actions = &current_game_.auto_actions;
    } else {
        actions = &current_game_.teleop_actions;
    }

    for (const auto& action : *actions) {
        // Check game piece requirement
        if (action.requires_game_piece && state_.held_pieces.empty()) {
            continue;
        }

        // Check proximity
        if (!action.valid_tags.empty()) {
            bool near_valid = false;
            for (int tag_id : action.valid_tags) {
                // Prefer visible tags
                bool is_visible = std::find(visible_tags.begin(), visible_tags.end(), tag_id) != visible_tags.end();
                if (is_visible || is_near_scoring_location(pose, tag_id, action.proximity_radius)) {
                    near_valid = true;
                    break;
                }
            }
            if (!near_valid) {
                continue;
            }
        }

        available.push_back(action);
    }

    return available;
}

std::vector<std::pair<GameYear, std::string>> GameScoringSystem::get_supported_games() {
    return {
        {GameYear::CRESCENDO_2024, "2024 CRESCENDO"},
        {GameYear::REEFSCAPE_2025, "2025 REEFSCAPE"},
        {GameYear::REBUILT_2026, "2026 REBUILT"}
    };
}

GameYear game_year_from_string(const std::string& str) {
    if (str == "2024" || str == "crescendo" || str == "CRESCENDO") {
        return GameYear::CRESCENDO_2024;
    } else if (str == "2025" || str == "reefscape" || str == "REEFSCAPE") {
        return GameYear::REEFSCAPE_2025;
    } else {
        return GameYear::REBUILT_2026;
    }
}

std::string game_year_to_string(GameYear year) {
    switch (year) {
        case GameYear::CRESCENDO_2024: return "2024 CRESCENDO";
        case GameYear::REEFSCAPE_2025: return "2025 REEFSCAPE";
        case GameYear::REBUILT_2026: return "2026 REBUILT";
        default: return "Unknown";
    }
}

} // namespace sim
} // namespace frc_vision
