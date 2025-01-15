#include "Engine/loader.h"
#include "Labs/2-GeometryProcessing/App.h"

namespace VCX::Labs::GeometryProcessing {
    using namespace Assets;

    App::App() :
        _casepc2mesh(_viewer),
        _ui(Labs::Common::UIOptions { }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
