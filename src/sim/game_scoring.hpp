#pragma once
/**
 * @file game_scoring.hpp
 * @brief FRC game scoring system for multiple game years
 *
 * Supports scoring for:
 * - 2024 CRESCENDO
 * - 2025 REEFSCAPE
 * - 2026 REBUILT
 */

#include "sim_types.hpp"
#include "../types.hpp"
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

namespace frc_vision {
namespace sim {

/**
 * @brief Game year enumeration
 */
enum class GameYear {
    CRESCENDO_2024,
    REEFSCAPE_2025,
    REBUILT_2026
};

/**
 * @brief Alliance color
 */
enum class Alliance {
    RED,
    BLUE
};

/**
 * @brief Scoring action for a specific game
 */
struct ScoringAction {
    std::string name;           // Human-readable name
    std::string code_name;      // Code-friendly identifier
    int points;                 // Points awarded
    std::vector<int> valid_tags; // Tags this action can occur at
    double proximity_radius;    // How close robot needs to be (meters)
    bool requires_game_piece;   // Whether robot needs to have a game piece
};

/**
 * @brief Game piece state
 */
struct GamePiece {
    std::string type;           // "note", "coral", "artifact", etc.
    bool held = false;
    double pickup_time = 0;
};

/**
 * @brief Match state tracking
 */
struct MatchState {
    GameYear game_year = GameYear::REBUILT_2026;
    Alliance alliance = Alliance::BLUE;

    // Match timing
    double match_time = 0;          // Seconds elapsed
    bool is_auto = true;            // Auto vs teleop
    bool is_endgame = false;        // Last 20 seconds

    // Score tracking
    int auto_score = 0;
    int teleop_score = 0;
    int endgame_score = 0;

    // Game piece tracking
    std::vector<GamePiece> held_pieces;
    int pieces_scored = 0;

    // Objectives completed
    std::vector<std::string> completed_objectives;

    int total_score() const { return auto_score + teleop_score + endgame_score; }
};

/**
 * @brief Game definition with scoring rules
 */
struct GameDefinition {
    GameYear year;
    std::string name;
    std::string field_layout_file;

    // Timing
    double auto_duration = 15.0;    // seconds
    double teleop_duration = 135.0; // seconds
    double endgame_start = 130.0;   // when endgame begins (from start)

    // Scoring actions
    std::vector<ScoringAction> auto_actions;
    std::vector<ScoringAction> teleop_actions;
    std::vector<ScoringAction> endgame_actions;

    // Game piece types
    std::vector<std::string> game_piece_types;

    // Starting positions
    std::vector<Pose2D> start_positions_red;
    std::vector<Pose2D> start_positions_blue;
};

/**
 * @brief Game scoring system
 */
class GameScoringSystem {
public:
    GameScoringSystem();

    /**
     * @brief Initialize with game year
     */
    void initialize(GameYear year, Alliance alliance);

    /**
     * @brief Get current game definition
     */
    const GameDefinition& get_game() const { return current_game_; }

    /**
     * @brief Get current match state
     */
    const MatchState& get_state() const { return state_; }
    MatchState& get_state() { return state_; }

    /**
     * @brief Update match time
     */
    void update(double dt);

    /**
     * @brief Reset match
     */
    void reset_match();

    /**
     * @brief Try to score at current position
     * @param pose Current robot pose
     * @param action_name Name of scoring action to attempt
     * @return Points scored (0 if invalid)
     */
    int try_score(const Pose2D& pose, const std::string& action_name);

    /**
     * @brief Check if robot is near a scoring location
     */
    bool is_near_scoring_location(const Pose2D& pose, int tag_id, double radius = 1.0) const;

    /**
     * @brief Pick up game piece
     */
    bool pickup_game_piece(const std::string& type);

    /**
     * @brief Get available scoring actions at current position
     */
    std::vector<ScoringAction> get_available_actions(const Pose2D& pose, const std::vector<int>& visible_tags) const;

    /**
     * @brief Get game piece count
     */
    int get_held_pieces() const { return static_cast<int>(state_.held_pieces.size()); }

    /**
     * @brief Get list of supported games
     */
    static std::vector<std::pair<GameYear, std::string>> get_supported_games();

private:
    void setup_crescendo_2024();
    void setup_reefscape_2025();
    void setup_rebuilt_2026();

    GameDefinition current_game_;
    MatchState state_;
    FieldLayout field_;
};

/**
 * @brief Get game year from string
 */
GameYear game_year_from_string(const std::string& str);

/**
 * @brief Get string from game year
 */
std::string game_year_to_string(GameYear year);

} // namespace sim
} // namespace frc_vision
