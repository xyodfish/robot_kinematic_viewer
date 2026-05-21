#include "kinematic_viewer/kinematic_playback.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace kinematic_viewer {
    namespace kinematic_playback_internal {

        std::string LowerString(std::string s);

        std::string NormalizeTrajectoryJointName(const std::string& raw_name) {
            static const std::unordered_map<std::string, std::string> kAliases = {
                {"leg1", "leg_joint1"},       {"leg2", "leg_joint2"},       {"leg3", "leg_joint3"},
                {"leg4", "leg_joint4"},       {"leg5", "leg_joint5"},       {"head1", "head_joint1"},
                {"head2", "head_joint2"},     {"left_arm1", "left_arm_joint1"}, {"left_arm2", "left_arm_joint2"},
                {"left_arm3", "left_arm_joint3"}, {"left_arm4", "left_arm_joint4"}, {"left_arm5", "left_arm_joint5"},
                {"left_arm6", "left_arm_joint6"}, {"left_arm7", "left_arm_joint7"}, {"right_arm1", "right_arm_joint1"},
                {"right_arm2", "right_arm_joint2"}, {"right_arm3", "right_arm_joint3"}, {"right_arm4", "right_arm_joint4"},
                {"right_arm5", "right_arm_joint5"}, {"right_arm6", "right_arm_joint6"}, {"right_arm7", "right_arm_joint7"},
            };
            const std::string key = LowerString(raw_name);
            const auto it         = kAliases.find(key);
            if (it != kAliases.end()) {
                return it->second;
            }
            return raw_name;
        }

        bool IsSkippedCsvColumn(const std::string& key_lower) {
            return key_lower == "idx" || key_lower == "id";
        }

        DebugPlaybackState::Mode NextPausedOrPlaying(DebugPlaybackState::Mode mode) {
            if (mode == DebugPlaybackState::Mode::Playing) {
                return DebugPlaybackState::Mode::Paused;
            }
            return DebugPlaybackState::Mode::Playing;
        }

        float WrapPhase(float phase) {
            const float twoPi = 6.283185307f;
            while (phase > twoPi) {
                phase -= twoPi;
            }
            while (phase < 0.0f) {
                phase += twoPi;
            }
            return phase;
        }

        bool ParseYamlScalarF32(const YAML::Node& node, float* out_value) {
            if (!node || !node.IsScalar() || out_value == nullptr) {
                return false;
            }
            try {
                const float value = node.as<float>();
                if (!std::isfinite(value)) {
                    return false;
                }
                *out_value = value;
                return true;
            } catch (const std::exception&) {
                return false;
            }
        }

        bool ParseBasePose2DMapNode(const YAML::Node& node, float* out_x_m, float* out_y_m, float* out_yaw_rad) {
            if (!node || !node.IsMap() || out_x_m == nullptr || out_y_m == nullptr || out_yaw_rad == nullptr) {
                return false;
            }
            float x_m = 0.0f;
            float y_m = 0.0f;
            if (!ParseYamlScalarF32(node["x"], &x_m) || !ParseYamlScalarF32(node["y"], &y_m)) {
                return false;
            }

            float yaw_rad = 0.0f;
            if (!ParseYamlScalarF32(node["yaw_rad"], &yaw_rad) && !ParseYamlScalarF32(node["yaw"], &yaw_rad)) {
                float yaw_deg = 0.0f;
                if (!ParseYamlScalarF32(node["yaw_deg"], &yaw_deg)) {
                    return false;
                }
                yaw_rad = yaw_deg * 0.017453292519943295f;
            }

            *out_x_m    = x_m;
            *out_y_m    = y_m;
            *out_yaw_rad = yaw_rad;
            return true;
        }

        bool ParseBasePose2DFlatNode(const YAML::Node& node, float* out_x_m, float* out_y_m, float* out_yaw_rad) {
            if (!node || !node.IsMap() || out_x_m == nullptr || out_y_m == nullptr || out_yaw_rad == nullptr) {
                return false;
            }
            float x_m = 0.0f;
            float y_m = 0.0f;
            if (!ParseYamlScalarF32(node["base_x"], &x_m) && !ParseYamlScalarF32(node["base_x_m"], &x_m)) {
                return false;
            }
            if (!ParseYamlScalarF32(node["base_y"], &y_m) && !ParseYamlScalarF32(node["base_y_m"], &y_m)) {
                return false;
            }

            float yaw_rad = 0.0f;
            if (!ParseYamlScalarF32(node["base_yaw_rad"], &yaw_rad) && !ParseYamlScalarF32(node["base_yaw"], &yaw_rad)) {
                float yaw_deg = 0.0f;
                if (!ParseYamlScalarF32(node["base_yaw_deg"], &yaw_deg)) {
                    return false;
                }
                yaw_rad = yaw_deg * 0.017453292519943295f;
            }

            *out_x_m    = x_m;
            *out_y_m    = y_m;
            *out_yaw_rad = yaw_rad;
            return true;
        }

        int FindClosestKeyframeByTimeSec(const std::vector<PoseKeyframe>& keyframes, double t_sec, double tol_sec) {
            if (keyframes.empty()) {
                return -1;
            }
            double best_dt = tol_sec;
            int best_index = -1;
            for (size_t i = 0; i < keyframes.size(); ++i) {
                const double dt = std::fabs(keyframes[i].t - t_sec);
                if (dt <= best_dt) {
                    best_dt    = dt;
                    best_index = static_cast<int>(i);
                }
            }
            return best_index;
        }

        bool MergeCompactBase2DTrack(const YAML::Node& trajectoryNode, std::vector<PoseKeyframe>* keyframes, std::string* errorMessage) {
            if (!trajectoryNode || !trajectoryNode.IsMap() || keyframes == nullptr) {
                return true;
            }
            const YAML::Node baseNode = trajectoryNode["base_2d"];
            if (!baseNode) {
                return true;
            }
            if (!baseNode.IsMap()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "compact base_2d: node must be map";
                }
                return false;
            }
            if (keyframes->empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "compact base_2d: no keyframes to attach base track";
                }
                return false;
            }

            const YAML::Node valuesNode  = baseNode["values"];
            const YAML::Node samplesNode = baseNode["samples"];
            if (valuesNode && valuesNode.IsSequence()) {
                if (!baseNode["dt"] || !baseNode["dt"].IsScalar()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "compact base_2d values: dt missing";
                    }
                    return false;
                }
                if (valuesNode.size() != keyframes->size()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "compact base_2d values: row count mismatch with keyframes";
                    }
                    return false;
                }
                for (size_t i = 0; i < valuesNode.size(); ++i) {
                    const YAML::Node& row = valuesNode[i];
                    if (!row || !row.IsSequence() || row.size() < 3) {
                        continue;
                    }
                    float x_m = 0.0f;
                    float y_m = 0.0f;
                    float yaw = 0.0f;
                    if (!ParseYamlScalarF32(row[0], &x_m) || !ParseYamlScalarF32(row[1], &y_m) || !ParseYamlScalarF32(row[2], &yaw)) {
                        continue;
                    }
                    auto& keyframe         = (*keyframes)[i];
                    keyframe.has_base_pose_2d = true;
                    keyframe.base_x_m      = x_m;
                    keyframe.base_y_m      = y_m;
                    keyframe.base_yaw_rad  = yaw;
                }
                return true;
            }
            if (samplesNode && samplesNode.IsSequence()) {
                bool assigned_any = false;
                for (const auto& row : samplesNode) {
                    if (!row || !row.IsSequence() || row.size() < 4) {
                        continue;
                    }
                    double t_sec = 0.0;
                    try {
                        t_sec = row[0].as<double>();
                    } catch (const std::exception&) {
                        continue;
                    }
                    float x_m = 0.0f;
                    float y_m = 0.0f;
                    float yaw = 0.0f;
                    if (!ParseYamlScalarF32(row[1], &x_m) || !ParseYamlScalarF32(row[2], &y_m) || !ParseYamlScalarF32(row[3], &yaw)) {
                        continue;
                    }
                    const int index = FindClosestKeyframeByTimeSec(*keyframes, t_sec, 1e-3);
                    if (index < 0) {
                        continue;
                    }
                    auto& keyframe         = (*keyframes)[static_cast<size_t>(index)];
                    keyframe.has_base_pose_2d = true;
                    keyframe.base_x_m      = x_m;
                    keyframe.base_y_m      = y_m;
                    keyframe.base_yaw_rad  = yaw;
                    assigned_any           = true;
                }
                if (!assigned_any && !samplesNode.IsNull() && samplesNode.size() > 0) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "compact base_2d samples: no rows matched keyframe time";
                    }
                    return false;
                }
                return true;
            }

            if (errorMessage != nullptr) {
                *errorMessage = "compact base_2d: require values or samples";
            }
            return false;
        }

        bool ParseLegacyKeyframesNode(const YAML::Node& keyframesNode, std::vector<PoseKeyframe>* outKeyframes) {
            if (!keyframesNode || !keyframesNode.IsSequence() || outKeyframes == nullptr) {
                return false;
            }
            std::vector<PoseKeyframe> loaded;
            loaded.reserve(keyframesNode.size());
            for (const auto& item : keyframesNode) {
                if (!item || !item.IsMap()) {
                    continue;
                }
                PoseKeyframe keyframe;
                keyframe.t        = item["t"] ? item["t"].as<double>() : 0.0;
                YAML::Node joints = item["joints"];
                if (joints && joints.IsMap()) {
                    for (auto it = joints.begin(); it != joints.end(); ++it) {
                        if (!it->first.IsScalar() || !it->second.IsScalar()) {
                            continue;
                        }
                        keyframe.joints[it->first.as<std::string>()] = it->second.as<float>();
                    }
                }
                float base_x_m = 0.0f;
                float base_y_m = 0.0f;
                float base_yaw_rad = 0.0f;
                if (ParseBasePose2DMapNode(item["base_pose_2d"], &base_x_m, &base_y_m, &base_yaw_rad) ||
                    ParseBasePose2DFlatNode(item, &base_x_m, &base_y_m, &base_yaw_rad)) {
                    keyframe.has_base_pose_2d = true;
                    keyframe.base_x_m         = base_x_m;
                    keyframe.base_y_m         = base_y_m;
                    keyframe.base_yaw_rad     = base_yaw_rad;
                }
                loaded.push_back(std::move(keyframe));
            }
            std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
            *outKeyframes = std::move(loaded);
            return true;
        }

        bool ParseCsvLikeYamlNode(const YAML::Node& rootNode, std::vector<PoseKeyframe>* outKeyframes, std::string* errorMessage) {
            if (!rootNode || !rootNode.IsMap() || outKeyframes == nullptr) {
                return false;
            }
            YAML::Node jointsNode = rootNode["joints"];
            YAML::Node valuesNode = rootNode["values"];
            if (!jointsNode || !jointsNode.IsSequence() || !valuesNode || !valuesNode.IsSequence()) {
                return false;
            }

            std::vector<std::string> columns;
            columns.reserve(jointsNode.size());
            for (const auto& joint : jointsNode) {
                if (!joint.IsScalar()) {
                    continue;
                }
                columns.push_back(joint.as<std::string>());
            }
            if (columns.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "csv-like yaml: joints is empty";
                }
                return false;
            }

            int base_x_col   = -1;
            int base_y_col   = -1;
            int base_yaw_col = -1;
            bool base_yaw_deg = false;
            std::vector<std::pair<std::string, size_t>> jointColumns;
            jointColumns.reserve(columns.size());
            for (size_t i = 0; i < columns.size(); ++i) {
                const std::string key = LowerString(columns[i]);
                if (IsSkippedCsvColumn(key)) {
                    continue;
                }
                if (key == "chassis_x" || key == "base_x" || key == "base_x_m" || key == "mobile_x") {
                    base_x_col = static_cast<int>(i);
                } else if (key == "chassis_y" || key == "base_y" || key == "base_y_m" || key == "mobile_y") {
                    base_y_col = static_cast<int>(i);
                } else if (key == "chassis_yaw_deg" || key == "base_yaw_deg") {
                    base_yaw_col = static_cast<int>(i);
                    base_yaw_deg = true;
                } else if (key == "chassis_yaw" || key == "chassis_z" || key == "base_yaw" || key == "base_yaw_rad" ||
                           key == "mobile_yaw") {
                    base_yaw_col = static_cast<int>(i);
                    base_yaw_deg = false;
                } else {
                    jointColumns.push_back({NormalizeTrajectoryJointName(columns[i]), i});
                }
            }
            const bool has_base_cols = (base_x_col >= 0 && base_y_col >= 0 && base_yaw_col >= 0);
            if (jointColumns.empty() && !has_base_cols) {
                if (errorMessage != nullptr) {
                    *errorMessage = "csv-like yaml: neither joints nor base columns found";
                }
                return false;
            }

            std::vector<PoseKeyframe> loaded;
            loaded.reserve(valuesNode.size());
            for (const auto& row : valuesNode) {
                if (!row || !row.IsSequence() || row.size() < (columns.size() + 1)) {
                    continue;
                }
                PoseKeyframe keyframe;
                try {
                    keyframe.t = row[0].as<double>();
                    for (const auto& [joint_name, col_idx] : jointColumns) {
                        keyframe.joints[joint_name] = row[col_idx + 1].as<float>();
                    }
                    if (has_base_cols) {
                        float base_x_m   = row[static_cast<size_t>(base_x_col) + 1].as<float>();
                        float base_y_m   = row[static_cast<size_t>(base_y_col) + 1].as<float>();
                        float base_yaw   = row[static_cast<size_t>(base_yaw_col) + 1].as<float>();
                        if (base_yaw_deg) {
                            base_yaw *= 0.017453292519943295f;
                        }
                        keyframe.has_base_pose_2d = true;
                        keyframe.base_x_m         = base_x_m;
                        keyframe.base_y_m         = base_y_m;
                        keyframe.base_yaw_rad     = base_yaw;
                    }
                } catch (const std::exception&) {
                    continue;
                }
                loaded.push_back(std::move(keyframe));
            }

            std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
            *outKeyframes = std::move(loaded);
            return !outKeyframes->empty();
        }

        bool ParseCompactSamplesNode(const YAML::Node& trajectoryNode, std::vector<PoseKeyframe>* outKeyframes, std::string* errorMessage) {
            if (!trajectoryNode || !trajectoryNode.IsMap() || outKeyframes == nullptr) {
                return false;
            }
            YAML::Node jointsNode  = trajectoryNode["joints"];
            YAML::Node samplesNode = trajectoryNode["samples"];
            if (!jointsNode || !jointsNode.IsSequence() || !samplesNode || !samplesNode.IsSequence()) {
                return false;
            }

            std::vector<std::string> jointNames;
            jointNames.reserve(jointsNode.size());
            for (const auto& joint : jointsNode) {
                if (!joint.IsScalar()) {
                    continue;
                }
                jointNames.push_back(joint.as<std::string>());
            }
            if (jointNames.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "compact samples: joints is empty";
                }
                return false;
            }

            std::vector<PoseKeyframe> loaded;
            loaded.reserve(samplesNode.size());
            for (const auto& row : samplesNode) {
                if (!row || !row.IsSequence() || row.size() < (jointNames.size() + 1)) {
                    continue;
                }
                PoseKeyframe keyframe;
                keyframe.t = row[0].as<double>();
                for (size_t i = 0; i < jointNames.size(); ++i) {
                    keyframe.joints[jointNames[i]] = row[i + 1].as<float>();
                }
                loaded.push_back(std::move(keyframe));
            }
            std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
            *outKeyframes = std::move(loaded);
            return true;
        }

        bool ParseCompactDtValuesNode(const YAML::Node& trajectoryNode, std::vector<PoseKeyframe>* outKeyframes,
                                      std::string* errorMessage) {
            if (!trajectoryNode || !trajectoryNode.IsMap() || outKeyframes == nullptr) {
                return false;
            }
            YAML::Node jointsNode = trajectoryNode["joints"];
            YAML::Node valuesNode = trajectoryNode["values"];
            YAML::Node dtNode     = trajectoryNode["dt"];
            if (!jointsNode || !jointsNode.IsSequence() || !valuesNode || !valuesNode.IsSequence() || !dtNode || !dtNode.IsScalar()) {
                return false;
            }

            const double dt = dtNode.as<double>();
            const double t0 = trajectoryNode["t0"] ? trajectoryNode["t0"].as<double>() : 0.0;
            if (dt <= 0.0) {
                if (errorMessage != nullptr) {
                    *errorMessage = "compact dt-values: dt must be > 0";
                }
                return false;
            }

            std::vector<std::string> jointNames;
            jointNames.reserve(jointsNode.size());
            for (const auto& joint : jointsNode) {
                if (!joint.IsScalar()) {
                    continue;
                }
                jointNames.push_back(joint.as<std::string>());
            }
            if (jointNames.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "compact dt-values: joints is empty";
                }
                return false;
            }

            std::vector<PoseKeyframe> loaded;
            loaded.reserve(valuesNode.size());
            for (size_t rowIndex = 0; rowIndex < valuesNode.size(); ++rowIndex) {
                const auto& row = valuesNode[rowIndex];
                if (!row || !row.IsSequence() || row.size() < jointNames.size()) {
                    continue;
                }
                PoseKeyframe keyframe;
                keyframe.t = t0 + dt * static_cast<double>(rowIndex);
                for (size_t i = 0; i < jointNames.size(); ++i) {
                    keyframe.joints[jointNames[i]] = row[i].as<float>();
                }
                loaded.push_back(std::move(keyframe));
            }

            *outKeyframes = std::move(loaded);
            return true;
        }

        std::vector<std::string> CollectJointNames(const DebugPlaybackState& playbackState) {
            std::vector<std::string> names;
            for (const auto& keyframe : playbackState.keyframes) {
                for (const auto& [jointName, _] : keyframe.joints) {
                    if (std::find(names.begin(), names.end(), jointName) == names.end()) {
                        names.push_back(jointName);
                    }
                }
            }

            // Preferred order for readability in trajectory yaml/csv:
            // leg -> head -> left_arm -> right_arm, with known robot-specific joint orders.
            const std::array<std::string, 12> leg_priority = {
                "left_hip_pitch_joint",  "left_hip_roll_joint",   "left_hip_yaw_joint",   "left_knee_joint",
                "left_ankle_pitch_joint","left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
                "right_hip_yaw_joint",   "right_knee_joint",      "right_ankle_pitch_joint","right_ankle_roll_joint",
            };
            const std::array<std::string, 3> head_priority = {
                "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            };
            const std::array<std::string, 7> left_arm_priority = {
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
                "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            };
            const std::array<std::string, 7> right_arm_priority = {
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
                "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
            };

            auto containsAny = [](const std::string& source, const std::initializer_list<const char*> tokens) {
                for (const char* token : tokens) {
                    if (source.find(token) != std::string::npos) {
                        return true;
                    }
                }
                return false;
            };

            auto groupRank = [&](const std::string& jointName) {
                const std::string lower = LowerString(jointName);
                if (containsAny(lower, {"leg_", "hip_", "knee_", "ankle_"})) {
                    return 0;
                }
                if (containsAny(lower, {"head_", "waist_"})) {
                    return 1;
                }
                if (containsAny(lower, {"left_arm_", "left_shoulder_", "left_elbow_", "left_wrist_"})) {
                    return 2;
                }
                if (containsAny(lower, {"right_arm_", "right_shoulder_", "right_elbow_", "right_wrist_"})) {
                    return 3;
                }
                return 4;
            };

            auto priorityIndex = [&](const std::string& jointName, const auto& arr) {
                for (size_t i = 0; i < arr.size(); ++i) {
                    if (arr[i] == jointName) {
                        return static_cast<int>(i);
                    }
                }
                return 1000000;
            };

            std::stable_sort(names.begin(), names.end(), [&](const std::string& a, const std::string& b) {
                const int ga = groupRank(a);
                const int gb = groupRank(b);
                if (ga != gb) {
                    return ga < gb;
                }

                if (ga == 0) {
                    const int ia = priorityIndex(a, leg_priority);
                    const int ib = priorityIndex(b, leg_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                } else if (ga == 1) {
                    const int ia = priorityIndex(a, head_priority);
                    const int ib = priorityIndex(b, head_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                } else if (ga == 2) {
                    const int ia = priorityIndex(a, left_arm_priority);
                    const int ib = priorityIndex(b, left_arm_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                } else if (ga == 3) {
                    const int ia = priorityIndex(a, right_arm_priority);
                    const int ib = priorityIndex(b, right_arm_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                }

                return LowerString(a) < LowerString(b);
            });
            return names;
        }

        bool EmitCsvLikeYaml(const DebugPlaybackState& playbackState, YAML::Emitter& emitter) {
            const std::vector<std::string> jointNames = CollectJointNames(playbackState);
            const bool hasAnyBase2d = std::any_of(playbackState.keyframes.begin(), playbackState.keyframes.end(),
                                                  [](const PoseKeyframe& keyframe) { return keyframe.has_base_pose_2d; });

            emitter << YAML::BeginMap;
            emitter << YAML::Key << "joints" << YAML::Value << YAML::BeginSeq;
            if (hasAnyBase2d) {
                emitter << "chassis_x" << "chassis_y" << "chassis_yaw";
            }
            for (const auto& jointName : jointNames) {
                emitter << jointName;
            }
            emitter << YAML::EndSeq;

            emitter << YAML::Key << "values" << YAML::Value << YAML::BeginSeq;
            std::unordered_map<std::string, float> lastValues;
            bool lastBaseValid = false;
            float lastBaseX      = 0.0f;
            float lastBaseY      = 0.0f;
            float lastBaseYaw    = 0.0f;
            for (const auto& keyframe : playbackState.keyframes) {
                emitter << YAML::BeginSeq;
                emitter << keyframe.t;
                if (hasAnyBase2d) {
                    if (keyframe.has_base_pose_2d) {
                        lastBaseValid = true;
                        lastBaseX     = keyframe.base_x_m;
                        lastBaseY     = keyframe.base_y_m;
                        lastBaseYaw   = keyframe.base_yaw_rad;
                    }
                    emitter << (lastBaseValid ? lastBaseX : 0.0f) << (lastBaseValid ? lastBaseY : 0.0f)
                            << (lastBaseValid ? lastBaseYaw : 0.0f);
                }
                for (const auto& jointName : jointNames) {
                    auto it = keyframe.joints.find(jointName);
                    if (it != keyframe.joints.end()) {
                        lastValues[jointName] = it->second;
                    }
                    float value = 0.0f;
                    auto last   = lastValues.find(jointName);
                    if (last != lastValues.end()) {
                        value = last->second;
                    }
                    emitter << value;
                }
                emitter << YAML::EndSeq;
            }
            emitter << YAML::EndSeq;
            emitter << YAML::EndMap;
            return true;
        }

        bool IsUniformTimeStep(const DebugPlaybackState& playbackState, double* outDt, double* outT0) {
            if (playbackState.keyframes.size() < 2 || outDt == nullptr || outT0 == nullptr) {
                return false;
            }
            const double dt = playbackState.keyframes[1].t - playbackState.keyframes[0].t;
            if (dt <= 0.0) {
                return false;
            }
            constexpr double kTol = 1e-6;
            for (size_t i = 2; i < playbackState.keyframes.size(); ++i) {
                const double step = playbackState.keyframes[i].t - playbackState.keyframes[i - 1].t;
                if (std::fabs(step - dt) > kTol) {
                    return false;
                }
            }
            *outDt = dt;
            *outT0 = playbackState.keyframes[0].t;
            return true;
        }

        std::string Trim(const std::string& input) {
            size_t begin = 0;
            while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
                ++begin;
            }
            size_t end = input.size();
            while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
                --end;
            }
            return input.substr(begin, end - begin);
        }

        std::vector<std::string> SplitCsvLine(const std::string& line) {
            std::vector<std::string> out;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                out.push_back(Trim(cell));
            }
            if (!line.empty() && line.back() == ',') {
                out.emplace_back("");
            }
            return out;
        }

        std::string LowerString(std::string s) {
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            return s;
        }

        bool EndsWithCaseInsensitive(const std::string& value, const std::string& suffix) {
            if (value.size() < suffix.size()) {
                return false;
            }
            const std::string left  = LowerString(value.substr(value.size() - suffix.size()));
            const std::string right = LowerString(suffix);
            return left == right;
        }

        std::string LowerFileExtension(const std::string& path) {
            std::filesystem::path p(path);
            std::string ext = p.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            return ext;
        }

    }  // namespace kinematic_playback_internal

    void LinearTrajectoryInterpolator::SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec,
                                                      int* currentSegmentIndex, omnilink::teleop_viewer::RobotScene* scene) const {
        auto lerpAngleRad = [](float a0, float a1, float alpha) {
            const float delta = std::atan2(std::sin(a1 - a0), std::cos(a1 - a0));
            return a0 + delta * alpha;
        };

        if (scene == nullptr || playbackState.keyframes.empty()) {
            return;
        }

        if (playbackState.keyframes.size() == 1) {
            const auto& only = playbackState.keyframes.front();
            for (const auto& [jointName, value] : only.joints) {
                scene->setJointPositionByName(jointName, value);
            }
            if (only.has_base_pose_2d) {
                scene->setVirtualBasePose2D(only.base_x_m, only.base_y_m, only.base_yaw_rad);
            }
            if (currentSegmentIndex != nullptr) {
                *currentSegmentIndex = 0;
            }
            return;
        }

        size_t hi = 1;
        while (hi < playbackState.keyframes.size() && static_cast<float>(playbackState.keyframes[hi].t) < sampleTimeSec) {
            ++hi;
        }
        size_t lo = (hi == 0) ? 0 : (hi - 1);
        hi        = std::min(hi, playbackState.keyframes.size() - 1);

        const auto& k0 = playbackState.keyframes[lo];
        const auto& k1 = playbackState.keyframes[hi];
        const float t0 = static_cast<float>(k0.t);
        const float t1 = static_cast<float>(k1.t);
        float alpha    = (t1 > t0 + 1e-6f) ? ((sampleTimeSec - t0) / (t1 - t0)) : 0.0f;
        alpha          = std::clamp(alpha, 0.0f, 1.0f);

        for (const auto& [jointName, value0] : k0.joints) {
            auto it1 = k1.joints.find(jointName);
            if (it1 == k1.joints.end()) {
                continue;
            }
            float value = value0 * (1.0f - alpha) + it1->second * alpha;
            scene->setJointPositionByName(jointName, value);
        }
        if (k0.has_base_pose_2d && k1.has_base_pose_2d) {
            const float x_m   = k0.base_x_m * (1.0f - alpha) + k1.base_x_m * alpha;
            const float y_m   = k0.base_y_m * (1.0f - alpha) + k1.base_y_m * alpha;
            const float yaw   = lerpAngleRad(k0.base_yaw_rad, k1.base_yaw_rad, alpha);
            scene->setVirtualBasePose2D(x_m, y_m, yaw);
        } else if (k0.has_base_pose_2d) {
            scene->setVirtualBasePose2D(k0.base_x_m, k0.base_y_m, k0.base_yaw_rad);
        } else if (k1.has_base_pose_2d) {
            scene->setVirtualBasePose2D(k1.base_x_m, k1.base_y_m, k1.base_yaw_rad);
        }

        if (currentSegmentIndex != nullptr) {
            *currentSegmentIndex = static_cast<int>(lo);
        }
    }

    TrajectoryPlayer::TrajectoryPlayer() : interpolator_(std::make_unique<LinearTrajectoryInterpolator>()) {}

    void TrajectoryPlayer::SetInterpolator(std::unique_ptr<TrajectoryInterpolator> interpolator) {
        if (interpolator) {
            interpolator_ = std::move(interpolator);
        }
    }

    void TrajectoryPlayer::RecordKeyframe(DebugPlaybackState* playbackState,
                                          const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints,
                                          const omnilink::teleop_viewer::RobotScene& scene) const {
        if (playbackState == nullptr) {
            return;
        }

        PoseKeyframe keyframe;
        if (!playbackState->keyframes.empty()) {
            keyframe.t = playbackState->keyframes.back().t + std::max(0.02f, playbackState->keyframe_interval_sec);
        }
        for (const auto& joint : joints) {
            if (!joint.revolute) {
                continue;
            }
            keyframe.joints[joint.name] = joint.position;
        }
        float base_x_m = 0.0f;
        float base_y_m = 0.0f;
        float base_yaw_rad = 0.0f;
        if (scene.getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw_rad)) {
            keyframe.has_base_pose_2d = true;
            keyframe.base_x_m         = base_x_m;
            keyframe.base_y_m         = base_y_m;
            keyframe.base_yaw_rad     = base_yaw_rad;
        }
        playbackState->keyframes.push_back(std::move(keyframe));
        playbackState->selected_keyframe_index = static_cast<int>(playbackState->keyframes.size()) - 1;
        playbackState->play_time               = static_cast<float>(playbackState->keyframes.back().t);
    }

    void TrajectoryPlayer::RemoveSelectedKeyframe(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr || playbackState->keyframes.empty()) {
            return;
        }
        const int index = playbackState->selected_keyframe_index;
        if (index < 0 || index >= static_cast<int>(playbackState->keyframes.size())) {
            return;
        }
        playbackState->keyframes.erase(playbackState->keyframes.begin() + index);
        if (playbackState->keyframes.empty()) {
            playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
            playbackState->selected_keyframe_index = -1;
            playbackState->play_time               = 0.0f;
            playbackState->current_segment_index   = -1;
            return;
        }
        playbackState->selected_keyframe_index = std::clamp(index, 0, static_cast<int>(playbackState->keyframes.size()) - 1);
        playbackState->play_time               = std::min(playbackState->play_time, TotalDuration(*playbackState));
    }

    void TrajectoryPlayer::Clear(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr) {
            return;
        }
        playbackState->keyframes.clear();
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->play_time               = 0.0f;
        playbackState->selected_keyframe_index = -1;
        playbackState->current_segment_index   = -1;
    }

    void TrajectoryPlayer::TogglePlayPause(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr || !HasPlayableTrajectory(*playbackState)) {
            return;
        }
        if (playbackState->mode == DebugPlaybackState::Mode::Stopped) {
            playbackState->play_time = 0.0f;
            playbackState->mode      = DebugPlaybackState::Mode::Playing;
            return;
        }
        playbackState->mode = kinematic_playback_internal::NextPausedOrPlaying(playbackState->mode);
    }

    void TrajectoryPlayer::Stop(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr) {
            return;
        }
        playbackState->mode      = DebugPlaybackState::Mode::Stopped;
        playbackState->play_time = 0.0f;
    }

    void TrajectoryPlayer::AdvanceAndApply(DebugPlaybackState* playbackState, omnilink::teleop_viewer::RobotScene* scene,
                                           double dtSec) const {
        if (playbackState == nullptr || scene == nullptr || !interpolator_) {
            return;
        }
        if (playbackState->mode != DebugPlaybackState::Mode::Playing || !HasPlayableTrajectory(*playbackState)) {
            return;
        }

        const float totalDuration = TotalDuration(*playbackState);
        if (totalDuration <= 1e-6f) {
            playbackState->mode = DebugPlaybackState::Mode::Stopped;
            return;
        }

        playbackState->play_time += static_cast<float>(dtSec) * playbackState->play_speed;
        if (playbackState->loop) {
            while (playbackState->play_time > totalDuration) {
                playbackState->play_time -= totalDuration;
            }
        } else if (playbackState->play_time > totalDuration) {
            playbackState->play_time = totalDuration;
            playbackState->mode      = DebugPlaybackState::Mode::Paused;
        }

        interpolator_->SampleAndApply(*playbackState, playbackState->play_time, &playbackState->current_segment_index, scene);
    }

    void TrajectoryPlayer::SampleAtCurrentTime(const DebugPlaybackState& playbackState, omnilink::teleop_viewer::RobotScene* scene) const {
        if (scene == nullptr || !interpolator_ || playbackState.keyframes.empty()) {
            return;
        }
        int segmentIndex = -1;
        interpolator_->SampleAndApply(playbackState, playbackState.play_time, &segmentIndex, scene);
    }

    float TrajectoryPlayer::TotalDuration(const DebugPlaybackState& playbackState) {
        if (playbackState.keyframes.empty()) {
            return 0.0f;
        }
        return static_cast<float>(playbackState.keyframes.back().t);
    }

    bool TrajectoryPlayer::HasPlayableTrajectory(const DebugPlaybackState& playbackState) {
        return playbackState.keyframes.size() >= 2;
    }

    bool LoadTrajectoryFromCsv(const std::string& csvPath, DebugPlaybackState* playbackState, std::string* errorMessage) {
        if (playbackState == nullptr) {
            if (errorMessage != nullptr) {
                *errorMessage = "playbackState is null";
            }
            return false;
        }
        std::ifstream file(csvPath);
        if (!file.good()) {
            if (errorMessage != nullptr) {
                *errorMessage = "open csv failed";
            }
            return false;
        }

        std::string line;
        std::vector<std::string> header;
        while (std::getline(file, line)) {
            const std::string stripped = kinematic_playback_internal::Trim(line);
            if (stripped.empty() || stripped[0] == '#') {
                continue;
            }
            header = kinematic_playback_internal::SplitCsvLine(stripped);
            break;
        }
        if (header.size() < 2) {
            if (errorMessage != nullptr) {
                *errorMessage = "csv header invalid, expected: time,joint1,...";
            }
            return false;
        }

        int time_col = -1;
        for (size_t i = 0; i < header.size(); ++i) {
            if (header[i].empty()) {
                continue;
            }
            const std::string key = kinematic_playback_internal::LowerString(header[i]);
            if (key == "time" || key == "t" || key == "timestamp") {
                time_col = static_cast<int>(i);
                break;
            }
        }
        if (time_col < 0) {
            if (errorMessage != nullptr) {
                *errorMessage = "csv missing time column (time/t/timestamp)";
            }
            return false;
        }

        std::vector<std::pair<std::string, size_t>> jointColumns;
        jointColumns.reserve(header.size() - 1);
        int base_x_col   = -1;
        int base_y_col   = -1;
        int base_yaw_col = -1;
        bool base_yaw_deg = false;
        for (size_t i = 0; i < header.size(); ++i) {
            if (static_cast<int>(i) == time_col || header[i].empty()) {
                continue;
            }
            const std::string key = kinematic_playback_internal::LowerString(header[i]);
            if (kinematic_playback_internal::IsSkippedCsvColumn(key)) {
                continue;
            }
            if (key == "chassis_x" || key == "base_x" || key == "base_x_m" || key == "mobile_x") {
                base_x_col = static_cast<int>(i);
            } else if (key == "chassis_y" || key == "base_y" || key == "base_y_m" || key == "mobile_y") {
                base_y_col = static_cast<int>(i);
            } else if (key == "chassis_yaw_deg" || key == "base_yaw_deg") {
                base_yaw_col = static_cast<int>(i);
                base_yaw_deg = true;
            } else if (key == "chassis_yaw" || key == "chassis_z" || key == "base_yaw" || key == "base_yaw_rad" ||
                       key == "mobile_yaw") {
                base_yaw_col = static_cast<int>(i);
                base_yaw_deg = false;
            } else {
                jointColumns.push_back({kinematic_playback_internal::NormalizeTrajectoryJointName(header[i]), i});
            }
        }
        if (jointColumns.empty()) {
            if (errorMessage != nullptr) {
                *errorMessage = "csv joints empty";
            }
            return false;
        }
        const bool has_base_cols = (base_x_col >= 0 && base_y_col >= 0 && base_yaw_col >= 0);

        std::vector<PoseKeyframe> loaded;
        while (std::getline(file, line)) {
            const std::string stripped = kinematic_playback_internal::Trim(line);
            if (stripped.empty() || stripped[0] == '#') {
                continue;
            }
            std::vector<std::string> cells = kinematic_playback_internal::SplitCsvLine(stripped);
            PoseKeyframe keyframe;
            bool row_ok = true;
            try {
                if (static_cast<size_t>(time_col) >= cells.size() || cells[time_col].empty()) {
                    row_ok = false;
                } else {
                    keyframe.t = std::stod(cells[time_col]);
                }
                for (const auto& [joint_name, col_idx] : jointColumns) {
                    if (col_idx >= cells.size() || cells[col_idx].empty()) {
                        row_ok = false;
                        break;
                    }
                    keyframe.joints[joint_name] = static_cast<float>(std::stod(cells[col_idx]));
                }
            } catch (const std::exception&) {
                row_ok = false;
            }
            if (!row_ok) {
                continue;
            }

            if (has_base_cols && static_cast<size_t>(base_x_col) < cells.size() && static_cast<size_t>(base_y_col) < cells.size() &&
                static_cast<size_t>(base_yaw_col) < cells.size() && !cells[base_x_col].empty() && !cells[base_y_col].empty() &&
                !cells[base_yaw_col].empty()) {
                try {
                    float base_x_m   = static_cast<float>(std::stod(cells[base_x_col]));
                    float base_y_m   = static_cast<float>(std::stod(cells[base_y_col]));
                    float base_yaw   = static_cast<float>(std::stod(cells[base_yaw_col]));
                    if (base_yaw_deg) {
                        base_yaw *= 0.017453292519943295f;
                    }
                    keyframe.has_base_pose_2d = true;
                    keyframe.base_x_m         = base_x_m;
                    keyframe.base_y_m         = base_y_m;
                    keyframe.base_yaw_rad     = base_yaw;
                } catch (const std::exception&) {
                }
            }
            loaded.push_back(std::move(keyframe));
        }

        std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
        playbackState->keyframes               = std::move(loaded);
        playbackState->selected_keyframe_index = playbackState->keyframes.empty() ? -1 : 0;
        playbackState->current_segment_index   = -1;
        playbackState->play_time               = 0.0f;
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->timeline_edited_this_ui = false;
        if (errorMessage != nullptr) {
            *errorMessage = "";
        }
        return !playbackState->keyframes.empty();
    }

    bool LoadTrajectoryFromYamlOnly(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage) {
        if (playbackState == nullptr) {
            if (errorMessage != nullptr) {
                *errorMessage = "playbackState is null";
            }
            return false;
        }
        try {
            YAML::Node root          = YAML::LoadFile(path);
            YAML::Node trajectory    = root["trajectory"];
            YAML::Node keyframesNode = root["keyframes"];
            if (!keyframesNode || !keyframesNode.IsSequence()) {
                keyframesNode = trajectory["keyframes"];
            }

            std::vector<PoseKeyframe> loaded;
            bool parsed = false;
            bool parsed_compact = false;
            if (kinematic_playback_internal::ParseCsvLikeYamlNode(root, &loaded, errorMessage)) {
                parsed = true;
            } else if (trajectory && kinematic_playback_internal::ParseCsvLikeYamlNode(trajectory, &loaded, errorMessage)) {
                parsed = true;
            } else if (kinematic_playback_internal::ParseLegacyKeyframesNode(keyframesNode, &loaded)) {
                parsed = true;
            } else if (kinematic_playback_internal::ParseCompactDtValuesNode(trajectory, &loaded, errorMessage)) {
                parsed        = true;
                parsed_compact = true;
            } else if (kinematic_playback_internal::ParseCompactSamplesNode(trajectory, &loaded, errorMessage)) {
                parsed        = true;
                parsed_compact = true;
            }
            if (!parsed) {
                if (errorMessage != nullptr && errorMessage->empty()) {
                    *errorMessage = "unsupported trajectory format";
                }
                return false;
            }
            if (parsed_compact && !kinematic_playback_internal::MergeCompactBase2DTrack(trajectory, &loaded, errorMessage)) {
                return false;
            }

            playbackState->keyframes               = std::move(loaded);
            playbackState->selected_keyframe_index = playbackState->keyframes.empty() ? -1 : 0;
            playbackState->current_segment_index   = -1;
            playbackState->play_time               = 0.0f;
            playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
            playbackState->timeline_edited_this_ui = false;
            if (errorMessage != nullptr) {
                *errorMessage = "";
            }
            return !playbackState->keyframes.empty();
        } catch (const std::exception& e) {
            if (errorMessage != nullptr) {
                *errorMessage = e.what();
            }
            return false;
        }
    }

    bool LoadTrajectoryFromFile(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage) {
        const std::string ext = kinematic_playback_internal::LowerFileExtension(path);
        if (ext == ".csv") {
            return LoadTrajectoryFromCsv(path, playbackState, errorMessage);
        }
        if (ext == ".yaml" || ext == ".yml") {
            return LoadTrajectoryFromYamlOnly(path, playbackState, errorMessage);
        }
        if (errorMessage != nullptr) {
            *errorMessage = "unsupported trajectory extension: " + ext + " (expected .yaml/.yml/.csv)";
        }
        return false;
    }

    bool SaveTrajectoryToFile(const std::string& path, const DebugPlaybackState& playbackState, std::string* errorMessage) {
        const std::string ext = kinematic_playback_internal::LowerFileExtension(path);
        if (ext == ".csv") {
            try {
                const std::vector<std::string> jointNames = kinematic_playback_internal::CollectJointNames(playbackState);
                const bool hasAnyBase2d = std::any_of(playbackState.keyframes.begin(), playbackState.keyframes.end(),
                                                      [](const PoseKeyframe& keyframe) { return keyframe.has_base_pose_2d; });
                std::ofstream file(path);
                if (!file.good()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "open csv failed";
                    }
                    return false;
                }

                file << "time";
                for (const auto& jointName : jointNames) {
                    file << "," << jointName;
                }
                if (hasAnyBase2d) {
                    file << ",chassis_x,chassis_y,chassis_yaw";
                }
                file << "\n";

                std::unordered_map<std::string, float> lastValues;
                bool lastBaseValid = false;
                float lastBaseX = 0.0f;
                float lastBaseY = 0.0f;
                float lastBaseYaw = 0.0f;
                for (const auto& keyframe : playbackState.keyframes) {
                    file << keyframe.t;
                    for (const auto& jointName : jointNames) {
                        auto it = keyframe.joints.find(jointName);
                        if (it != keyframe.joints.end()) {
                            lastValues[jointName] = it->second;
                        }
                        float value = 0.0f;
                        auto last   = lastValues.find(jointName);
                        if (last != lastValues.end()) {
                            value = last->second;
                        }
                        file << "," << value;
                    }
                    if (hasAnyBase2d) {
                        if (keyframe.has_base_pose_2d) {
                            lastBaseValid = true;
                            lastBaseX     = keyframe.base_x_m;
                            lastBaseY     = keyframe.base_y_m;
                            lastBaseYaw   = keyframe.base_yaw_rad;
                        }
                        if (lastBaseValid) {
                            file << "," << lastBaseX << "," << lastBaseY << "," << lastBaseYaw;
                        } else {
                            file << ",0,0,0";
                        }
                    }
                    file << "\n";
                }
                file.close();
                if (errorMessage != nullptr) {
                    *errorMessage = "";
                }
                return true;
            } catch (const std::exception& e) {
                if (errorMessage != nullptr) {
                    *errorMessage = e.what();
                }
                return false;
            }
        }
        if (!(ext == ".yaml" || ext == ".yml")) {
            if (errorMessage != nullptr) {
                *errorMessage = "unsupported trajectory extension: " + ext + " (expected .yaml/.yml/.csv)";
            }
            return false;
        }
        try {
            YAML::Emitter out;
            kinematic_playback_internal::EmitCsvLikeYaml(playbackState, out);

            std::ofstream file(path);
            if (!file.good()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "open file failed";
                }
                return false;
            }
            file << out.c_str();
            file.close();
            if (errorMessage != nullptr) {
                *errorMessage = "";
            }
            return true;
        } catch (const std::exception& e) {
            if (errorMessage != nullptr) {
                *errorMessage = e.what();
            }
            return false;
        }
    }

    bool LoadTrajectoryFromYaml(const std::string& yamlPath, DebugPlaybackState* playbackState, std::string* errorMessage) {
        return LoadTrajectoryFromYamlOnly(yamlPath, playbackState, errorMessage);
    }

    bool SaveTrajectoryToYaml(const std::string& yamlPath, const DebugPlaybackState& playbackState, std::string* errorMessage) {
        return SaveTrajectoryToFile(yamlPath, playbackState, errorMessage);
    }

    void BuildDemoTrajectoryFromCurrentPose(DebugPlaybackState* playbackState,
                                            const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints,
                                            const omnilink::teleop_viewer::RobotScene& scene) {
        if (playbackState == nullptr) {
            return;
        }
        playbackState->keyframes.clear();
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->play_time               = 0.0f;
        playbackState->selected_keyframe_index = -1;
        playbackState->current_segment_index   = -1;

        std::vector<omnilink::teleop_viewer::RobotScene::JointInfo> revoluteJoints;
        revoluteJoints.reserve(joints.size());
        for (const auto& joint : joints) {
            if (joint.revolute) {
                revoluteJoints.push_back(joint);
            }
        }
        if (revoluteJoints.empty()) {
            return;
        }

        const int frameCount = 16;
        const float dt       = std::max(0.1f, playbackState->keyframe_interval_sec);
        const float twoPi    = 6.283185307f;
        float start_base_x_m = 0.0f;
        float start_base_y_m = 0.0f;
        float start_base_yaw = 0.0f;
        const bool has_base_pose = scene.getVirtualBasePose2D(&start_base_x_m, &start_base_y_m, &start_base_yaw);
        for (int frame = 0; frame < frameCount; ++frame) {
            PoseKeyframe keyframe;
            keyframe.t = static_cast<double>(frame) * static_cast<double>(dt);
            const float phase =
                kinematic_playback_internal::WrapPhase((static_cast<float>(frame) / static_cast<float>(frameCount - 1)) * twoPi);
            for (size_t i = 0; i < revoluteJoints.size(); ++i) {
                const auto& joint            = revoluteJoints[i];
                const float jointRange       = std::max(0.0f, joint.max_angle - joint.min_angle);
                const float amplitudeByRange = std::max(0.03f, std::min(0.25f, 0.12f * jointRange));
                const float amplitude        = std::min(amplitudeByRange, 0.35f);
                const float offsetPhase      = phase + static_cast<float>(i) * 0.35f;
                float value                  = joint.position + amplitude * std::sin(offsetPhase);
                value                        = std::clamp(value, joint.min_angle, joint.max_angle);
                keyframe.joints[joint.name]  = value;
            }
            if (has_base_pose) {
                const float x_wave = 0.06f * std::sin(phase);
                const float y_wave = 0.04f * std::cos(phase);
                const float yaw_wave = 0.20f * std::sin(phase);
                keyframe.has_base_pose_2d = true;
                keyframe.base_x_m         = start_base_x_m + x_wave;
                keyframe.base_y_m         = start_base_y_m + y_wave;
                keyframe.base_yaw_rad     = start_base_yaw + yaw_wave;
            }
            playbackState->keyframes.push_back(std::move(keyframe));
        }

        playbackState->selected_keyframe_index = 0;
    }

}  // namespace kinematic_viewer
