
#include <algorithm>
#include <array>

#include "Labs/2-GeometryProcessing/Casepc2mesh.h"
#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GeometryProcessing {


    Casepc2mesh::Casepc2mesh(Viewer & viewer):
        _viewer(viewer) {
        _cameraManager.EnablePan       = false;
        _cameraManager.AutoRotateSpeed = 0.f;
        _options.LightDirection = glm::vec3(glm::cos(glm::radians(_options.LightDirScalar)), -1.0f, glm::sin(glm::radians(_options.LightDirScalar)));
    }

    void Casepc2mesh::OnSetupPropsUI() {
        if (ImGui::BeginCombo("Geometry", _geometryTypeName[static_cast<std::uint32_t>(_type)].data())) {
            for (std::uint32_t i = 0; i < _geometryTypeName.size(); ++i) {
                bool selected = (static_cast<std::uint32_t>(_type) == i);
                if (ImGui::Selectable(_geometryTypeName[i].data(), selected)) {
                    if (! selected) {
                        _type      = ImplicitGeometryType(i);
                        _recompute = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        Common::ImGuiHelper::SaveImage(_viewer.GetTexture(), _viewer.GetSize(), true);
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            _recompute |= ImGui::SliderInt("Resolution", &_resolution, -10, 100);
            if (_running) {
                static const std::string t = "Running.....";
                ImGui::Text(t.substr(0, 7 + (static_cast<int>(ImGui::GetTime() / 0.1f) % 6)).c_str());
            } else ImGui::NewLine();
        }
        ImGui::Spacing();

        Viewer::SetupRenderOptionsUI(_options, _cameraManager);
    }

    Common::CaseRenderResult Casepc2mesh::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _task.Emplace([&]() {
                Engine::SurfaceMesh emptyMesh;
                if (_type == ImplicitGeometryType::square)
                    pc2mesh(emptyMesh, "/home/daihang/pc2mesh/square.ply",  0.25*(1+_resolution/100.0));
                else if (_type == ImplicitGeometryType::Torus)
                    pc2mesh(emptyMesh, "/home/daihang/pc2mesh/cylinder.ply" , 0.09*(1+_resolution/100.0));
                return emptyMesh;
            });
            _running = true;
        }
        if (_running && _task.HasValue()) {
            _running = false;
            _modelObject.ReplaceMesh(_task.Value());
        }
        return _viewer.Render(_options, _modelObject, _camera, _cameraManager, desiredSize);
    }

    void Casepc2mesh::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }
} // namespace VCX::Labs::GeometryProcessing
