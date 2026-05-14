/**
 * @file vrr_vpr.cpp
 * @brief Implementation of the C++ VPREngine wrapper around the Python module.
 *
 * Uses pybind11 embedded interpreter to call into the vrr_vpr Python package.
 * The Python interpreter is started once (on first VPREngine construction)
 * and kept alive for the process lifetime.
 */

#include "vrr_vpr/vrr_vpr.h"

#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <mutex>
#include <stdexcept>

namespace py = pybind11;

namespace vrr {

// ---------------------------------------------------------------------------
// Interpreter lifetime management
// ---------------------------------------------------------------------------

namespace {

std::once_flag g_interp_flag;
bool g_owns_interpreter = false;

void ensure_interpreter() {
    std::call_once(g_interp_flag, [] {
        if (!Py_IsInitialized()) {
            py::initialize_interpreter();
            g_owns_interpreter = true;
        }
    });
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// PIMPL
// ---------------------------------------------------------------------------

class VPREngineImpl {
public:
    VPREngineImpl(const std::string& method,
                  const std::string& device,
                  bool               rerank,
                  int                max_keypoints,
                  int                rerank_resize,
                  bool               color_rerank,
                  float              color_weight)
    {
        ensure_interpreter();
        py::gil_scoped_acquire gil;

        py::module_ vrr_mod = py::module_::import("vrr_vpr");
        engine_ = vrr_mod.attr("VPREngine")(
            py::arg("method")        = method,
            py::arg("device")        = device,
            py::arg("rerank")        = rerank,
            py::arg("max_keypoints") = max_keypoints,
            py::arg("rerank_resize") = rerank_resize,
            py::arg("color_rerank")  = color_rerank,
            py::arg("color_weight")  = color_weight
        );
        descriptor_dim_ = engine_.attr("descriptor_dim").cast<int>();
    }

    int buildDatabase(const std::string& folder, int bs, int nw) {
        py::gil_scoped_acquire gil;
        return engine_.attr("build_database")(folder, bs, nw).cast<int>();
    }

    int buildDatabaseFromDescriptors(const float* data, int n, int dim,
                                     const std::vector<std::string>& paths) {
        py::gil_scoped_acquire gil;
        auto arr = py::array_t<float>({n, dim},           // shape
                                      {dim * sizeof(float), sizeof(float)}, // strides
                                      data);               // pointer (no copy)
        return engine_.attr("build_database_from_descriptors")(arr, paths)
                      .cast<int>();
    }

    int addToDatabase(const std::string& path) {
        py::gil_scoped_acquire gil;
        return engine_.attr("add_to_database")(path).cast<int>();
    }

    int addImageToDatabase(const uint8_t* data, int h, int w,
                           const std::string& label) {
        py::gil_scoped_acquire gil;
        auto arr = py::array_t<uint8_t>(
            {h, w, 3},
            {static_cast<py::ssize_t>(w * 3), py::ssize_t{3}, py::ssize_t{1}},
            data
        );
        return engine_.attr("add_image_to_database")(arr, label).cast<int>();
    }

    int databaseSize() const {
        py::gil_scoped_acquire gil;
        return engine_.attr("database_size")().cast<int>();
    }

    QueryResult query(const std::string& path, int top_k) const {
        py::gil_scoped_acquire gil;
        py::dict res = engine_.attr("query")(path, top_k);
        return parseDictWithDistances(res);
    }

    QueryResult queryImage(const uint8_t* data, int h, int w,
                           int top_k) const {
        py::gil_scoped_acquire gil;
        auto arr = py::array_t<uint8_t>(
            {h, w, 3},
            {static_cast<py::ssize_t>(w * 3), py::ssize_t{3}, py::ssize_t{1}},
            data
        );
        py::dict res = engine_.attr("query_image")(arr, top_k);
        return parseDictWithDistances(res);
    }

    QueryResult queryWithRerank(const std::string& path,
                                int top_k, int rerank_k) const {
        py::gil_scoped_acquire gil;
        py::dict res = engine_.attr("query_with_rerank")(path, top_k, rerank_k);
        return parseDictWithScores(res);
    }

    QueryResult queryWithColorRerank(const std::string& path,
                                     int top_k, int candidates_k) const {
        py::gil_scoped_acquire gil;
        py::dict res = engine_.attr("query_with_color_rerank")(
            path, top_k, candidates_k
        );
        return parseDictWithScores(res);
    }

    void saveDatabase(const std::string& dir) const {
        py::gil_scoped_acquire gil;
        engine_.attr("save_database")(dir);
    }

    int loadDatabase(const std::string& dir) {
        py::gil_scoped_acquire gil;
        return engine_.attr("load_database")(dir).cast<int>();
    }

    void extractDescriptor(const std::string& path, float* out) const {
        py::gil_scoped_acquire gil;
        py::array_t<float> arr =
            engine_.attr("extract_descriptor")(path).cast<py::array_t<float>>();
        auto buf = arr.unchecked<1>();
        for (py::ssize_t i = 0; i < buf.shape(0); ++i)
            out[i] = buf(i);
    }

    void extractDescriptorFromImage(const uint8_t* data, int h, int w,
                                    float* out) const {
        py::gil_scoped_acquire gil;
        auto img = py::array_t<uint8_t>(
            {h, w, 3},
            {static_cast<py::ssize_t>(w * 3), py::ssize_t{3}, py::ssize_t{1}},
            data
        );
        py::array_t<float> arr =
            engine_.attr("extract_descriptor_from_image")(img)
                   .cast<py::array_t<float>>();
        auto buf = arr.unchecked<1>();
        for (py::ssize_t i = 0; i < buf.shape(0); ++i)
            out[i] = buf(i);
    }

    int descriptorDim() const { return descriptor_dim_; }

private:
    py::object engine_;
    int descriptor_dim_;

    static QueryResult parseDictWithDistances(const py::dict& d) {
        QueryResult r;
        r.indices   = d["indices"].cast<std::vector<int>>();
        r.distances = d["distances"].cast<std::vector<float>>();
        r.paths     = d["paths"].cast<std::vector<std::string>>();
        return r;
    }

    static QueryResult parseDictWithScores(const py::dict& d) {
        QueryResult r;
        r.indices = d["indices"].cast<std::vector<int>>();
        r.scores  = d["scores"].cast<std::vector<float>>();
        r.paths   = d["paths"].cast<std::vector<std::string>>();
        return r;
    }
};

// ---------------------------------------------------------------------------
// VPREngine forwarding
// ---------------------------------------------------------------------------

VPREngine::VPREngine(const std::string& method,
                     const std::string& device,
                     bool               rerank,
                     int                max_keypoints,
                     int                rerank_resize,
                     bool               color_rerank,
                     float              color_weight)
    : impl_(std::make_unique<VPREngineImpl>(
          method, device, rerank, max_keypoints, rerank_resize,
          color_rerank, color_weight))
{}

VPREngine::~VPREngine() {
    // Release the Python object while the GIL is held.
    if (impl_) {
        py::gil_scoped_acquire gil;
        impl_.reset();
    }
}

VPREngine::VPREngine(VPREngine&&) noexcept            = default;
VPREngine& VPREngine::operator=(VPREngine&&) noexcept = default;

int VPREngine::buildDatabase(const std::string& folder,
                             int batch_size, int num_workers) {
    return impl_->buildDatabase(folder, batch_size, num_workers);
}

int VPREngine::buildDatabaseFromDescriptors(
        const float* descriptors, int n, int dim,
        const std::vector<std::string>& paths) {
    return impl_->buildDatabaseFromDescriptors(descriptors, n, dim, paths);
}

int VPREngine::addToDatabase(const std::string& path) {
    return impl_->addToDatabase(path);
}

int VPREngine::addImageToDatabase(const uint8_t* data, int h, int w,
                                  const std::string& label) {
    return impl_->addImageToDatabase(data, h, w, label);
}

int VPREngine::databaseSize() const {
    return impl_->databaseSize();
}

QueryResult VPREngine::query(const std::string& path, int top_k) const {
    return impl_->query(path, top_k);
}

QueryResult VPREngine::queryImage(const uint8_t* data, int h, int w,
                                  int top_k) const {
    return impl_->queryImage(data, h, w, top_k);
}

QueryResult VPREngine::queryWithRerank(const std::string& path,
                                       int top_k, int rerank_k) const {
    return impl_->queryWithRerank(path, top_k, rerank_k);
}

QueryResult VPREngine::queryWithColorRerank(const std::string& path,
                                            int top_k, int candidates_k) const {
    return impl_->queryWithColorRerank(path, top_k, candidates_k);
}

void VPREngine::saveDatabase(const std::string& dir) const {
    impl_->saveDatabase(dir);
}

int VPREngine::loadDatabase(const std::string& dir) {
    return impl_->loadDatabase(dir);
}

void VPREngine::extractDescriptor(const std::string& path,
                                  float* out) const {
    impl_->extractDescriptor(path, out);
}

void VPREngine::extractDescriptorFromImage(const uint8_t* data,
                                           int h, int w,
                                           float* out) const {
    impl_->extractDescriptorFromImage(data, h, w, out);
}

int VPREngine::descriptorDim() const {
    return impl_->descriptorDim();
}

}  // namespace vrr
