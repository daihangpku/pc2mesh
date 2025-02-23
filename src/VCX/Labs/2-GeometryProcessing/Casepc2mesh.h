#pragma once

#include <array>
#include <string_view>

#include "Engine/Async.hpp"
#include "Labs/2-GeometryProcessing/Viewer.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::GeometryProcessing {
    class Casepc2mesh : public Common::ICase {
    public:
        enum class ImplicitGeometryType {
            square = 0,
            Torus
        };

        std::array<std::string_view, 2> _geometryTypeName {
            "square",
            "cylinder"
        };

        Casepc2mesh(Viewer & viewer);

        virtual std::string_view const GetName() override { return "pc2mesh"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;


    private:
        Engine::Async<Engine::SurfaceMesh> _task;
        Viewer &                           _viewer;
        Engine::Camera                     _camera        { .Eye = glm::vec3(-1, 1, 1), };
        Common::OrbitCameraManager         _cameraManager { glm::vec3(-1, 1, 1) };
        ImplicitGeometryType               _type          { ImplicitGeometryType::square };
        bool                               _recompute     { true };
        bool                               _running       { false };
        ModelObject                        _modelObject;
        RenderOptions                      _options;
        int                               _resolution    { 1 };
    };
} // namespace VCX::Labs::GeometryProcessing
