#include "kinematic_viewer/kinematic_sidebar_panels.h"

#include "kinematic_viewer/kinematic_user_obstacles.h"

#include "imgui.h"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace kinematic_viewer {
    namespace kinematic_sidebar_panels_internal {

        std::string NormalizePath(const std::string& path) {
            std::error_code ec;
            std::filesystem::path p(path);
            auto normalized = std::filesystem::weakly_canonical(p, ec);
            if (!ec) {
                return normalized.string();
            }
            return p.lexically_normal().string();
        }

        bool IsTrajectoryFileExt(const std::filesystem::path& path) {
            std::string ext = path.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            return ext == ".yaml" || ext == ".yml" || ext == ".csv";
        }

        bool ValidateTrajectoryJointNames(const DebugPlaybackState& playbackState,
                                          const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints,
                                          std::string* errorMessage) {
            std::unordered_set<std::string> sceneJointNames;
            for (const auto& joint : joints) {
                sceneJointNames.insert(joint.name);
            }

            std::unordered_set<std::string> trajectoryJointNames;
            for (const auto& keyframe : playbackState.keyframes) {
                for (const auto& [jointName, _] : keyframe.joints) {
                    trajectoryJointNames.insert(jointName);
                }
            }

            if (trajectoryJointNames.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "轨迹文件中未找到任何关节名";
                }
                return false;
            }

            std::vector<std::string> unknown;
            int matchedCount = 0;
            for (const auto& jointName : trajectoryJointNames) {
                if (sceneJointNames.find(jointName) == sceneJointNames.end()) {
                    unknown.push_back(jointName);
                } else {
                    ++matchedCount;
                }
            }

            if (!unknown.empty()) {
                std::sort(unknown.begin(), unknown.end());
                std::stringstream ss;
                ss << "轨迹关节名与当前机器人不匹配，未知关节 " << unknown.size() << " 个: ";
                const size_t showCount = std::min<size_t>(unknown.size(), 8);
                for (size_t i = 0; i < showCount; ++i) {
                    if (i > 0) {
                        ss << ", ";
                    }
                    ss << unknown[i];
                }
                if (unknown.size() > showCount) {
                    ss << " ...";
                }
                if (errorMessage != nullptr) {
                    *errorMessage = ss.str();
                }
                return false;
            }

            if (matchedCount <= 0) {
                if (errorMessage != nullptr) {
                    *errorMessage = "轨迹关节名与当前机器人无任何匹配";
                }
                return false;
            }

            return true;
        }

        std::string TrimCopy(const std::string& text) {
            size_t begin = 0;
            while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
                ++begin;
            }
            size_t end = text.size();
            while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
                --end;
            }
            return text.substr(begin, end - begin);
        }

        std::vector<std::string> SplitJointGroupCommands(const std::string& rawText) {
            std::string normalized = rawText;
            std::replace(normalized.begin(), normalized.end(), ';', '\n');
            std::vector<std::string> lines;
            std::stringstream ss(normalized);
            std::string line;
            while (std::getline(ss, line)) {
                const std::string trimmed = TrimCopy(line);
                if (!trimmed.empty()) {
                    lines.push_back(trimmed);
                }
            }
            return lines;
        }

        bool ParseJointValuesRad(const std::string& rawValues, std::vector<float>* outValues, std::string* errorMessage) {
            if (outValues == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：输出缓冲为空";
                }
                return false;
            }
            std::string normalized = rawValues;
            for (char& ch : normalized) {
                if (ch == ',' || ch == '[' || ch == ']' || ch == '\t') {
                    ch = ' ';
                }
            }
            std::stringstream ss(normalized);
            std::vector<float> values;
            float value = 0.0f;
            while (ss >> value) {
                if (!std::isfinite(value)) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "存在非有限数值(NaN/Inf)";
                    }
                    return false;
                }
                values.push_back(value);
            }
            if (values.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "未解析到任何关节角数值";
                }
                return false;
            }
            *outValues = std::move(values);
            return true;
        }

        const ViewerState::JointInputGroup* FindJointInputGroup(const ViewerState& uiState, const std::string& groupNameRaw) {
            std::string target = groupNameRaw;
            std::transform(target.begin(), target.end(), target.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            for (const auto& group : uiState.joint_input_groups) {
                std::string current = group.name;
                std::transform(current.begin(), current.end(), current.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (current == target) {
                    return &group;
                }
            }
            return nullptr;
        }

        bool ApplyJointGroupTextCommand(const std::string& commandLine, const ViewerState& uiState,
                                        omnilink::teleop_viewer::RobotScene* scene, std::string* errorMessage) {
            if (scene == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：scene为空";
                }
                return false;
            }
            const std::string line = TrimCopy(commandLine);
            if (line.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "空输入";
                }
                return false;
            }

            size_t firstSpace = line.find_first_of(" \t");
            if (firstSpace == std::string::npos) {
                if (errorMessage != nullptr) {
                    *errorMessage = "格式错误，应为: group v1,v2,...";
                }
                return false;
            }
            const std::string groupName = line.substr(0, firstSpace);
            const std::string valuesRaw = TrimCopy(line.substr(firstSpace + 1));
            if (valuesRaw.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "缺少关节角列表";
                }
                return false;
            }

            const ViewerState::JointInputGroup* group = FindJointInputGroup(uiState, groupName);
            if (group == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "未知group: " + groupName;
                }
                return false;
            }
            if (group->joint_names.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "group未配置关节名: " + groupName;
                }
                return false;
            }

            std::vector<float> values;
            std::string parseError;
            if (!ParseJointValuesRad(valuesRaw, &values, &parseError)) {
                if (errorMessage != nullptr) {
                    *errorMessage = parseError;
                }
                return false;
            }
            if (values.size() != group->joint_names.size()) {
                if (errorMessage != nullptr) {
                    std::stringstream ss;
                    ss << "group[" << group->name << "] 维度不匹配，期望 " << group->joint_names.size() << "，实际 " << values.size();
                    *errorMessage = ss.str();
                }
                return false;
            }

            for (size_t i = 0; i < group->joint_names.size(); ++i) {
                if (!scene->setJointPositionByName(group->joint_names[i], values[i])) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "场景不存在关节: " + group->joint_names[i];
                    }
                    return false;
                }
            }
            return true;
        }

        bool ApplyJointGroupValues(const ViewerState::JointInputGroup& group, const std::vector<float>& values,
                                   omnilink::teleop_viewer::RobotScene* scene, std::string* errorMessage) {
            if (scene == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：scene为空";
                }
                return false;
            }
            if (group.joint_names.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "group未配置关节名: " + group.name;
                }
                return false;
            }
            if (values.size() != group.joint_names.size()) {
                if (errorMessage != nullptr) {
                    std::stringstream ss;
                    ss << "group[" << group.name << "] 维度不匹配，期望 " << group.joint_names.size() << "，实际 " << values.size();
                    *errorMessage = ss.str();
                }
                return false;
            }
            for (size_t i = 0; i < group.joint_names.size(); ++i) {
                if (!scene->setJointPositionByName(group.joint_names[i], values[i])) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "场景不存在关节: " + group.joint_names[i];
                    }
                    return false;
                }
            }
            return true;
        }

        void RenderTrajectoryFileBrowser(DebugPlaybackState* playbackState) {
            if (playbackState == nullptr) {
                return;
            }

            if (ImGui::Button("浏览本地文件")) {
                const std::string defaultDir = NormalizePath(std::filesystem::current_path().string());
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                              defaultDir.c_str());
                ImGui::OpenPopup("trajectory_file_browser_popup");
            }

            if (!ImGui::BeginPopupModal("trajectory_file_browser_popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                return;
            }

            ImGui::InputText("目录", playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir));
            ImGui::SameLine();
            if (ImGui::Button("进入目录")) {
                const std::string normalized = NormalizePath(playbackState->trajectory_browser_dir);
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                              normalized.c_str());
            }
            ImGui::SameLine();
            if (ImGui::Button("上一级")) {
                std::filesystem::path current = std::filesystem::path(NormalizePath(playbackState->trajectory_browser_dir));
                std::filesystem::path parent  = current.parent_path();
                if (parent.empty()) {
                    parent = std::filesystem::path("/");
                }
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                              parent.string().c_str());
            }
            ImGui::SameLine();
            if (ImGui::Button("根目录/")) {
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", "/");
            }
            ImGui::SameLine();
            if (ImGui::Button("HOME")) {
                const char* home = std::getenv("HOME");
                if (home != nullptr && home[0] != '\0') {
                    std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", home);
                }
            }
            ImGui::TextDisabled("支持输入任意绝对路径，例如 /home/user/data");

            std::error_code ec;
            const std::filesystem::path browsePath(playbackState->trajectory_browser_dir);
            if (!std::filesystem::exists(browsePath, ec) || !std::filesystem::is_directory(browsePath, ec)) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "目录不可用");
            } else {
                std::vector<std::filesystem::path> dirs;
                std::vector<std::filesystem::path> files;
                for (auto it = std::filesystem::directory_iterator(browsePath, ec); !ec && it != std::filesystem::directory_iterator();
                     ++it) {
                    if (it->is_directory(ec)) {
                        dirs.push_back(it->path());
                    } else if (it->is_regular_file(ec) && IsTrajectoryFileExt(it->path())) {
                        files.push_back(it->path());
                    }
                }
                std::sort(dirs.begin(), dirs.end());
                std::sort(files.begin(), files.end());

                if (ImGui::BeginChild("trajectory_file_browser_list", ImVec2(580, 280), true)) {
                    for (const auto& d : dirs) {
                        std::string label = "[DIR] " + d.filename().string();
                        if (ImGui::Selectable(label.c_str(), false)) {
                            std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                                          d.string().c_str());
                        }
                    }
                    for (const auto& f : files) {
                        std::string label = f.filename().string();
                        if (ImGui::Selectable(label.c_str(), false)) {
                            const std::string selectedPath = f.string();
                            std::snprintf(playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path), "%s",
                                          selectedPath.c_str());
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndChild();
                }
            }

            if (ImGui::Button("关闭")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

    }  // namespace kinematic_sidebar_panels_internal

    void RenderScenePanel(ViewerState* uiState) {
        if (uiState == nullptr) {
            return;
        }
        ImGui::Checkbox("显示关节轴", &uiState->show_axes);
        ImGui::Checkbox("仅旋转关节轴", &uiState->show_revolute_only);
        ImGui::Checkbox("显示非旋转关节", &uiState->show_non_revolute);
        ImGui::Checkbox("显示世界坐标轴", &uiState->show_world_axes);
        ImGui::Checkbox("固定底座模式", &uiState->lock_base);
        ImGui::SliderFloat("关节轴长度", &uiState->axis_length, 0.03f, 0.5f, "%.3f");
        ImGui::SliderFloat("线宽", &uiState->axis_line_width, 1.0f, 6.0f, "%.1f");
        ImGui::SliderFloat("世界轴长度", &uiState->world_axis_length, 0.1f, 1.5f, "%.2f");
        ImGui::SliderFloat("地面网格尺寸", &uiState->grid_size, 1.0f, 20.0f, "%.1f");
        ImGui::SliderInt("地面网格密度", &uiState->grid_count, 10, 120);
    }

    void RenderObstaclePanel(ViewerState* uiState) {
        if (uiState == nullptr) {
            return;
        }
        ImGui::Separator();
        RenderUserObstaclePanel(&uiState->user_obstacles);
    }

    void RenderJointPanel(ViewerState* uiState, omnilink::teleop_viewer::RobotScene* scene,
                          const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) {
        if (uiState == nullptr || scene == nullptr) {
            return;
        }

        int revoluteCount   = 0;
        int clampedCount    = 0;
        float minMarginDeg  = 1e9f;
        std::string minName = "";
        for (const auto& j : joints) {
            if (j.revolute) {
                ++revoluteCount;
            }
            if (j.position < j.min_angle - 1e-5f || j.position > j.max_angle + 1e-5f) {
                ++clampedCount;
            }
            if (j.revolute) {
                float d0 = std::fabs(j.position - j.min_angle);
                float d1 = std::fabs(j.max_angle - j.position);
                float m  = glm::degrees(std::min(d0, d1));
                if (m < minMarginDeg) {
                    minMarginDeg = m;
                    minName      = j.name;
                }
            }
        }

        ImGui::InputText("关节过滤", uiState->joint_filter, sizeof(uiState->joint_filter));
        std::string filter = uiState->joint_filter;
        std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        ImGui::Text("关节总数: %d  旋转关节: %d  越界关节: %d", static_cast<int>(joints.size()), revoluteCount, clampedCount);
        if (!minName.empty()) {
            ImVec4 c = (minMarginDeg < 3.0f) ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)
                                             : ((minMarginDeg < 8.0f) ? ImVec4(1.0f, 0.75f, 0.25f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f));
            ImGui::TextColored(c, "最小限位裕量: %.2f deg (%s)", minMarginDeg, minName.c_str());
        }

        ImGui::Separator();
        ImGui::TextUnformatted("分组批量输入(rad)");
        if (uiState->joint_input_groups.empty()) {
            ImGui::TextDisabled("当前未配置可批量输入的group（请在 config.initial_pose.*_joint_names 配置）。");
        } else {
            if (uiState->selected_joint_input_group < 0 ||
                uiState->selected_joint_input_group >= static_cast<int>(uiState->joint_input_groups.size())) {
                uiState->selected_joint_input_group = 0;
            }
            std::vector<const char*> groupNames;
            groupNames.reserve(uiState->joint_input_groups.size());
            for (const auto& group : uiState->joint_input_groups) {
                groupNames.push_back(group.name.c_str());
            }
            ImGui::Combo("Group下拉", &uiState->selected_joint_input_group, groupNames.data(), static_cast<int>(groupNames.size()));

            const auto& selectedGroup = uiState->joint_input_groups[static_cast<size_t>(uiState->selected_joint_input_group)];
            ImGui::Text("当前Group: %s (关节数: %d)", selectedGroup.name.c_str(), static_cast<int>(selectedGroup.joint_names.size()));
            ImGui::TextDisabled("当前Group输入格式: -0.90,1.24,...（只填角度，不需要group名）");
            ImGui::InputTextMultiline("当前Group角度(rad)", uiState->joint_group_values_input, sizeof(uiState->joint_group_values_input),
                                      ImVec2(-FLT_MIN, 72.0f));
            if (ImGui::Button("应用当前Group输入")) {
                std::vector<float> values;
                std::string error;
                if (!kinematic_sidebar_panels_internal::ParseJointValuesRad(uiState->joint_group_values_input, &values, &error)) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                } else if (!kinematic_sidebar_panels_internal::ApplyJointGroupValues(selectedGroup, values, scene, &error)) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                } else {
                    uiState->joint_group_input_last_ok = true;
                    uiState->joint_group_input_status  = "应用成功: " + selectedGroup.name;
                }
            }

            ImGui::Separator();
            std::stringstream groupDesc;
            groupDesc << "可用group: ";
            for (size_t i = 0; i < uiState->joint_input_groups.size(); ++i) {
                if (i > 0) {
                    groupDesc << " | ";
                }
                groupDesc << uiState->joint_input_groups[i].name << "(" << uiState->joint_input_groups[i].joint_names.size() << ")";
            }
            ImGui::TextWrapped("%s", groupDesc.str().c_str());
            ImGui::TextDisabled("示例: right_arm -0.903373,1.24978,1.93879,2.29756,-1.84481,0.135545,-1.0195");
            ImGui::TextDisabled("支持多条: 每行一条，或使用 ';' 分隔。");
            ImGui::InputTextMultiline("##joint_group_input", uiState->joint_group_input, sizeof(uiState->joint_group_input),
                                      ImVec2(-FLT_MIN, 84.0f));
            if (ImGui::Button("应用分组关节输入")) {
                const std::string rawText               = uiState->joint_group_input;
                const std::vector<std::string> commands = kinematic_sidebar_panels_internal::SplitJointGroupCommands(rawText);
                if (commands.empty()) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "输入为空";
                } else {
                    bool allOk  = true;
                    int okCount = 0;
                    std::vector<std::string> failures;
                    failures.reserve(commands.size());
                    for (size_t i = 0; i < commands.size(); ++i) {
                        std::string error;
                        if (kinematic_sidebar_panels_internal::ApplyJointGroupTextCommand(commands[i], *uiState, scene, &error)) {
                            ++okCount;
                        } else {
                            allOk = false;
                            std::stringstream ss;
                            ss << "第" << (i + 1) << "条失败: " << error;
                            failures.push_back(ss.str());
                        }
                    }
                    uiState->joint_group_input_last_ok = allOk;
                    std::stringstream status;
                    status << "已应用 " << okCount << "/" << commands.size() << " 条";
                    if (!failures.empty()) {
                        status << " | " << failures.front();
                    }
                    uiState->joint_group_input_status = status.str();
                }
            }
            if (!uiState->joint_group_input_status.empty()) {
                const ImVec4 statusColor =
                    uiState->joint_group_input_last_ok ? ImVec4(0.45f, 0.95f, 0.45f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f);
                ImGui::TextColored(statusColor, "%s", uiState->joint_group_input_status.c_str());
            }
        }

        if (ImGui::Button("旋转关节全部归零")) {
            for (const auto& j : joints) {
                if (j.revolute) {
                    scene->setJointPositionByName(j.name, 0.0f);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("一键夹紧到限位内")) {
            for (const auto& j : joints) {
                if (!j.revolute) {
                    continue;
                }
                float v = std::clamp(j.position, j.min_angle, j.max_angle);
                scene->setJointPositionByName(j.name, v);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("保存当前姿态")) {
            uiState->pose_snapshot.clear();
            for (const auto& j : joints) {
                uiState->pose_snapshot[j.name] = j.position;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("恢复保存姿态") && !uiState->pose_snapshot.empty()) {
            for (const auto& [name, value] : uiState->pose_snapshot) {
                scene->setJointPositionByName(name, value);
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("关节调试");
        if (ImGui::BeginTable("joint_table", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
            ImGui::TableSetupColumn("关节");
            ImGui::TableSetupColumn("滑条(deg)");
            ImGui::TableSetupColumn("滑条(rad)");
            ImGui::TableSetupColumn("限位");
            ImGui::TableSetupColumn("输入(deg)");
            ImGui::TableSetupColumn("输入(rad)");
            ImGui::TableHeadersRow();

            for (const auto& j : joints) {
                if (!uiState->show_non_revolute && !j.revolute) {
                    continue;
                }
                std::string nameLower = j.name;
                std::transform(nameLower.begin(), nameLower.end(), nameLower.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (!filter.empty() && nameLower.find(filter) == std::string::npos) {
                    continue;
                }

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(j.name.c_str());
                ImGui::TableSetColumnIndex(1);
                float radValue = j.position;
                if (j.revolute) {
                    ImGui::PushItemWidth(-1);
                    std::string sliderDegId = "##slider_deg_" + j.name;
                    if (ImGui::SliderAngle(sliderDegId.c_str(), &radValue, glm::degrees(j.min_angle), glm::degrees(j.max_angle))) {
                        scene->setJointPositionByName(j.name, radValue);
                    }
                    ImGui::PopItemWidth();
                } else {
                    ImGui::TextDisabled("不适用");
                }

                ImGui::TableSetColumnIndex(2);
                if (j.revolute) {
                    ImGui::PushItemWidth(-1);
                    std::string sliderRadId = "##slider_rad_" + j.name;
                    if (ImGui::SliderFloat(sliderRadId.c_str(), &radValue, j.min_angle, j.max_angle, "%.4f")) {
                        scene->setJointPositionByName(j.name, radValue);
                    }
                    ImGui::PopItemWidth();
                } else {
                    ImGui::TextDisabled("不适用");
                }

                ImGui::TableSetColumnIndex(3);
                if (j.revolute) {
                    ImGui::Text("deg: %.2f ~ %.2f", glm::degrees(j.min_angle), glm::degrees(j.max_angle));
                    ImGui::Text("rad: %.4f ~ %.4f", j.min_angle, j.max_angle);
                } else {
                    ImGui::TextDisabled("不适用");
                }

                ImGui::TableSetColumnIndex(4);
                if (j.revolute) {
                    float inputDeg         = glm::degrees(j.position);
                    std::string inputDegId = "##input_deg_" + j.name;
                    if (ImGui::InputFloat(inputDegId.c_str(), &inputDeg, 0.1f, 1.0f, "%.2f")) {
                        scene->setJointPositionByName(j.name, glm::radians(inputDeg));
                    }
                } else {
                    ImGui::TextDisabled("不适用");
                }

                ImGui::TableSetColumnIndex(5);
                if (j.revolute) {
                    float inputRad         = j.position;
                    std::string inputRadId = "##input_rad_" + j.name;
                    if (ImGui::InputFloat(inputRadId.c_str(), &inputRad, 0.01f, 0.1f, "%.4f")) {
                        scene->setJointPositionByName(j.name, inputRad);
                    }
                } else {
                    ImGui::TextDisabled("不适用");
                }
            }
            ImGui::EndTable();
        }
    }

    void RenderPlaybackPanel(DebugPlaybackState* playbackState, TrajectoryPlayer* playbackPlayer,
                             omnilink::teleop_viewer::RobotScene* scene,
                             const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) {
        if (playbackState == nullptr || playbackPlayer == nullptr || scene == nullptr) {
            return;
        }

        constexpr const char* kTrajectoryAlertPopupId = "轨迹告警##trajectory_incompatible_alert_popup";
        if (playbackState->trajectory_alert_popup_pending) {
            ImGui::OpenPopup(kTrajectoryAlertPopupId);
            playbackState->trajectory_alert_popup_pending = false;
        }
        ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f), ImGuiCond_Appearing);
        if (ImGui::BeginPopupModal(kTrajectoryAlertPopupId, nullptr, ImGuiWindowFlags_NoResize)) {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.22f, 1.0f), "轨迹文件不适用");
            ImGui::Separator();
            ImGui::TextWrapped("%s", playbackState->trajectory_alert_message.empty() ? "该轨迹无法用于当前机器人。"
                                                                                     : playbackState->trajectory_alert_message.c_str());
            if (!playbackState->trajectory_alert_detail.empty() && ImGui::CollapsingHeader("查看详情")) {
                ImGui::TextWrapped("%s", playbackState->trajectory_alert_detail.c_str());
            }
            ImGui::Spacing();
            if (ImGui::Button("知道了", ImVec2(120.0f, 0.0f))) {
                playbackState->trajectory_alert_detail.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::Separator();
        ImGui::TextUnformatted("轨迹关键帧回放");

        if (ImGui::CollapsingHeader("文件", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::DragFloat("关键帧间隔(s)", &playbackState->keyframe_interval_sec, 0.02f, 0.02f, 5.0f, "%.2f");
            ImGui::InputText("轨迹文件", playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path));
            ImGui::SameLine();
            kinematic_sidebar_panels_internal::RenderTrajectoryFileBrowser(playbackState);

            if (ImGui::Button("加载轨迹文件")) {
                const DebugPlaybackState previousState = *playbackState;
                std::string ioError;
                if (LoadTrajectoryFromFile(playbackState->trajectory_file_path, playbackState, &ioError)) {
                    std::string checkError;
                    if (!kinematic_sidebar_panels_internal::ValidateTrajectoryJointNames(*playbackState, joints, &checkError)) {
                        *playbackState                                = previousState;
                        playbackState->trajectory_io_status           = "加载失败: " + checkError;
                        playbackState->trajectory_alert_message       = "该轨迹与当前机器人关节定义不匹配。";
                        playbackState->trajectory_alert_detail        = checkError;
                        playbackState->trajectory_alert_popup_pending = true;
                    } else {
                        playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                        playbackState->trajectory_io_status = "加载成功";
                    }
                } else {
                    playbackState->trajectory_io_status           = "加载失败: " + ioError;
                    playbackState->trajectory_alert_message       = "轨迹文件加载失败，请检查路径或文件格式。";
                    playbackState->trajectory_alert_detail        = ioError;
                    playbackState->trajectory_alert_popup_pending = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("保存当前轨迹")) {
                std::string ioError;
                if (SaveTrajectoryToFile(playbackState->trajectory_file_path, *playbackState, &ioError)) {
                    playbackState->trajectory_io_status = "保存成功";
                } else {
                    playbackState->trajectory_io_status = "保存失败: " + ioError;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("生成Demo轨迹")) {
                BuildDemoTrajectoryFromCurrentPose(playbackState, joints);
                std::string ioError;
                if (SaveTrajectoryToFile(playbackState->trajectory_file_path, *playbackState, &ioError)) {
                    playbackState->trajectory_io_status = "Demo轨迹已生成并保存";
                } else {
                    playbackState->trajectory_io_status = "Demo生成成功但保存失败: " + ioError;
                }
                playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
            }

            if (!playbackState->trajectory_io_status.empty()) {
                ImVec4 color(0.66f, 0.72f, 0.80f, 1.0f);
                if (playbackState->trajectory_io_status.find("失败") != std::string::npos) {
                    color = ImVec4(0.95f, 0.42f, 0.42f, 1.0f);
                } else if (playbackState->trajectory_io_status.find("成功") != std::string::npos) {
                    color = ImVec4(0.40f, 0.84f, 0.52f, 1.0f);
                }
                ImGui::TextColored(color, "%s", playbackState->trajectory_io_status.c_str());
            }
        }

        if (ImGui::CollapsingHeader("播放控制", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("记录关键帧")) {
                playbackPlayer->RecordKeyframe(playbackState, joints);
            }
            ImGui::SameLine();
            const bool playing = playbackState->mode == DebugPlaybackState::Mode::Playing;
            if (ImGui::Button(playing ? "暂停回放" : "开始回放")) {
                playbackPlayer->TogglePlayPause(playbackState);
            }
            ImGui::SameLine();
            if (ImGui::Button("停止")) {
                playbackPlayer->Stop(playbackState);
            }
            ImGui::SameLine();
            if (ImGui::Button("清空")) {
                playbackPlayer->Clear(playbackState);
            }

            ImGui::Checkbox("循环回放", &playbackState->loop);
            ImGui::SliderFloat("回放倍速", &playbackState->play_speed, 0.1f, 3.0f, "%.2fx");

            if (!playbackState->keyframes.empty()) {
                float total = TrajectoryPlayer::TotalDuration(*playbackState);
                if (ImGui::SliderFloat("回放时间", &playbackState->play_time, 0.0f, std::max(0.0f, total), "%.2f s")) {
                    playbackState->timeline_edited_this_ui = true;
                }
                if (playbackState->timeline_edited_this_ui) {
                    playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                    playbackState->timeline_edited_this_ui = false;
                }

                const char* modeLabel = "Stopped";
                if (playbackState->mode == DebugPlaybackState::Mode::Playing) {
                    modeLabel = "Playing";
                } else if (playbackState->mode == DebugPlaybackState::Mode::Paused) {
                    modeLabel = "Paused";
                }
                ImGui::Text("状态: %s  总时长: %.2fs  当前段: %d", modeLabel, total, playbackState->current_segment_index);
                ImGui::Text("关键帧数: %d", static_cast<int>(playbackState->keyframes.size()));
            } else {
                ImGui::TextDisabled("暂无关键帧，点击“记录关键帧”开始。");
            }
        }

        if (ImGui::CollapsingHeader("关键帧列表", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!playbackState->keyframes.empty() && playbackState->selected_keyframe_index >= 0 &&
                playbackState->selected_keyframe_index < static_cast<int>(playbackState->keyframes.size())) {
                if (ImGui::Button("删除选中关键帧")) {
                    playbackPlayer->RemoveSelectedKeyframe(playbackState);
                }
            }

            if (playbackState->keyframes.empty()) {
                ImGui::TextDisabled("暂无关键帧。");
            } else if (ImGui::BeginTable("keyframe_table", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                                         ImVec2(0.0f, 200.0f))) {
                ImGui::TableSetupColumn("索引");
                ImGui::TableSetupColumn("时间(s)");
                ImGui::TableSetupColumn("关节数");
                ImGui::TableHeadersRow();
                for (int i = 0; i < static_cast<int>(playbackState->keyframes.size()); ++i) {
                    const auto& keyframe = playbackState->keyframes[static_cast<size_t>(i)];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    char selectLabel[32];
                    snprintf(selectLabel, sizeof(selectLabel), "KF %d", i);
                    if (ImGui::Selectable(selectLabel, playbackState->selected_keyframe_index == i, ImGuiSelectableFlags_SpanAllColumns)) {
                        playbackState->selected_keyframe_index = i;
                        playbackState->play_time               = static_cast<float>(keyframe.t);
                        playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                    }
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.2f", keyframe.t);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%d", static_cast<int>(keyframe.joints.size()));
                }
                ImGui::EndTable();
            }
        }
    }

    void RenderSafetyPanel(CollisionMonitorState* collisionState, const CollisionMonitorResult& collisionResult) {
        if (collisionState == nullptr) {
            return;
        }
        ImGui::Separator();
        ImGui::TextUnformatted("碰撞预警与距离监控");
        ImGui::Checkbox("启用碰撞监控", &collisionState->enable);
        ImGui::SameLine();
        ImGui::Checkbox("显示最近对连线", &collisionState->show_closest_pair_line);
        ImGui::Checkbox("忽略同一Link", &collisionState->ignore_same_link);
        ImGui::SameLine();
        ImGui::Checkbox("忽略父子Link", &collisionState->ignore_parent_child);
        ImGui::SetNextItemWidth(180.0f);
        ImGui::DragFloat("Danger阈值(m)", &collisionState->danger_distance_m, 0.002f, -0.20f, 0.30f, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(180.0f);
        ImGui::DragFloat("Warning阈值(m)", &collisionState->warning_distance_m, 0.002f, -0.20f, 0.50f, "%.3f");
        if (collisionState->warning_distance_m < collisionState->danger_distance_m) {
            collisionState->warning_distance_m = collisionState->danger_distance_m;
        }

        if (ImGui::BeginTable("safety_stats", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("评估Pair数");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("Warning对数");
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("Danger对数");
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", collisionState->evaluated_pair_count);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d", collisionResult.warning_pair_count);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d", collisionResult.danger_pair_count);
            ImGui::EndTable();
        }
        if (!collisionState->has_valid_distance) {
            ImGui::TextDisabled("暂无可用距离数据（可能proxy不足或全部被过滤）");
            return;
        }

        ImVec4 color(0.60f, 0.95f, 0.60f, 1.0f);
        if (collisionState->nearest_surface_distance_m <= collisionState->danger_distance_m) {
            color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
        } else if (collisionState->nearest_surface_distance_m <= collisionState->warning_distance_m) {
            color = ImVec4(1.0f, 0.80f, 0.30f, 1.0f);
        }

        ImGui::Text("最近Link对: %s <-> %s", collisionState->nearest_link_a.c_str(), collisionState->nearest_link_b.c_str());
        ImGui::TextColored(color, "最近表面距离: %.3f m", collisionState->nearest_surface_distance_m);
        ImGui::Text("中心距离: %.3f m", collisionState->nearest_center_distance_m);
    }

    void RenderTfPanel(ViewerState* uiState, const std::vector<omnilink::teleop_viewer::RobotScene::LinkTfInfo>& tfs) {
        if (uiState == nullptr) {
            return;
        }
        ImGui::Separator();
        ImGui::TextUnformatted("TF 视图");
        ImGui::InputText("TF过滤", uiState->tf_filter, sizeof(uiState->tf_filter));
        std::string tfFilter = uiState->tf_filter;
        std::transform(tfFilter.begin(), tfFilter.end(), tfFilter.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        if (ImGui::BeginTable("tf_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
            ImGui::TableSetupColumn("Link");
            ImGui::TableSetupColumn("父Link");
            ImGui::TableSetupColumn("位置 xyz(m)");
            ImGui::TableSetupColumn("姿态 rpy(deg)");
            ImGui::TableHeadersRow();
            for (const auto& tf : tfs) {
                std::string key      = tf.name + " " + tf.parent_name;
                std::string keyLower = key;
                std::transform(keyLower.begin(), keyLower.end(), keyLower.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (!tfFilter.empty() && keyLower.find(tfFilter) == std::string::npos) {
                    continue;
                }
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(tf.name.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(tf.parent_name.empty() ? "-" : tf.parent_name.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3f, %.3f, %.3f", tf.world_position.x, tf.world_position.y, tf.world_position.z);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.1f, %.1f, %.1f", glm::degrees(tf.world_rpy.x), glm::degrees(tf.world_rpy.y), glm::degrees(tf.world_rpy.z));
            }
            ImGui::EndTable();
        }
    }

}  // namespace kinematic_viewer
