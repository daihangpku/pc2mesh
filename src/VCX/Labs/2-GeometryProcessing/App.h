#pragma once

#include <vector>

#include "Assets/bundled.h"
#include "Engine/app.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/Casepc2mesh.h"

#include "Labs/Common/UI.h"

namespace VCX::Labs::GeometryProcessing {
    class App : public Engine::IApp {
    private:
        Common::UI           _ui;
        Viewer               _viewer;
        Casepc2mesh    _casepc2mesh;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _casepc2mesh,
        };

    public:
        App();

        void OnFrame() override;
    };
}
