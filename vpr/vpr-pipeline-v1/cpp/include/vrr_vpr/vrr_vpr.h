/**
 * @file vrr_vpr.h
 * @brief C++ API for the vrr_vpr Visual Place Recognition engine.
 *
 * Wraps the Python vrr_vpr module via pybind11 embedded interpreter.
 * The heavy lifting (MegaLoc/SALAD-indoor, FAISS, SuperPoint+LightGlue,
 * color re-scoring) runs in Python/PyTorch; this header provides a
 * native C++ interface.
 *
 * Usage:
 *   vrr::VPREngine engine("salad-indoor", "cuda", false, 1024, 512, true);
 *   engine.buildDatabase("/path/to/database/images");
 *   auto result = engine.queryWithColorRerank("/path/to/query.jpg", 5);
 *   for (auto& p : result.paths) std::cout << p << "\n";
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vrr {

/// Result of a VPR query.
struct QueryResult {
    std::vector<int>         indices;    ///< Database indices of matches
    std::vector<float>       distances;  ///< L2 distances  (query/queryImage)
    std::vector<float>       scores;     ///< Inlier counts (queryWithRerank only)
    std::vector<std::string> paths;      ///< File paths of matched db images
};

/// Forward-declare the PIMPL implementation.
class VPREngineImpl;

/**
 * Visual Place Recognition engine.
 *
 * Manages a Python interpreter internally.  Only one VPREngine instance
 * should exist per process (Python GIL constraint).
 */
class VPREngine {
public:
    /**
     * Construct the engine, loading the specified model.
     *
     * @param method        Model name: "megaloc" or "salad-indoor".
     * @param device        "cuda" or "cpu".
     * @param rerank        Pre-load SuperPoint + LightGlue for geometric
     *                      re-ranking queries.
     * @param max_keypoints Max SuperPoint keypoints (rerank only).
     * @param rerank_resize Max image dimension for SuperPoint (rerank only).
     * @param color_rerank  Enable HSV color histogram re-scoring.
     * @param color_weight  Alpha for color scoring formula.
     */
    explicit VPREngine(const std::string& method        = "megaloc",
                       const std::string& device        = "cuda",
                       bool               rerank        = false,
                       int                max_keypoints  = 1024,
                       int                rerank_resize  = 512,
                       bool               color_rerank   = false,
                       float              color_weight   = 5.0f);

    ~VPREngine();

    // Non-copyable, movable.
    VPREngine(const VPREngine&)            = delete;
    VPREngine& operator=(const VPREngine&) = delete;
    VPREngine(VPREngine&&) noexcept;
    VPREngine& operator=(VPREngine&&) noexcept;

    // ----------------------------------------------------------------
    // Database management
    // ----------------------------------------------------------------

    /**
     * Build the FAISS index from a folder of images.
     * @return Number of images indexed.
     */
    int buildDatabase(const std::string& database_folder,
                      int batch_size  = 4,
                      int num_workers = 4);

    /**
     * Build from pre-computed descriptors and paths.
     * @param descriptors  Row-major float32 array, shape (N, descriptor_dim).
     * @param num_images   Number of rows.
     * @param descriptor_dim  Descriptor dimensionality (8448 for MegaLoc).
     * @param image_paths  Corresponding image paths (size N).
     * @return Number of images indexed.
     */
    int buildDatabaseFromDescriptors(const float* descriptors,
                                     int num_images,
                                     int descriptor_dim,
                                     const std::vector<std::string>& image_paths);

    /**
     * Add a single image to the live database.
     * @return The new database index.
     */
    int addToDatabase(const std::string& image_path);

    /**
     * Add a raw image buffer to the database.
     * @param data     Pointer to RGB uint8 pixel data (row-major, 3 channels).
     * @param height   Image height.
     * @param width    Image width.
     * @param label    Identifier stored as the "path" for this entry.
     * @return The new database index.
     */
    int addImageToDatabase(const uint8_t* data,
                           int height, int width,
                           const std::string& label = "");

    /// Number of images in the database.
    int databaseSize() const;

    // ----------------------------------------------------------------
    // Querying
    // ----------------------------------------------------------------

    /// Query by image file path.
    QueryResult query(const std::string& image_path, int top_k = 5) const;

    /**
     * Query with a raw image buffer (RGB uint8, row-major).
     * @param data     Pointer to pixel data.
     * @param height   Image height.
     * @param width    Image width.
     * @param top_k    Number of results to return.
     */
    QueryResult queryImage(const uint8_t* data,
                           int height, int width,
                           int top_k = 5) const;

    /**
     * Query + geometric re-ranking (SuperPoint + LightGlue).
     * Requires `rerank=true` in the constructor.
     */
    QueryResult queryWithRerank(const std::string& image_path,
                                int top_k    = 5,
                                int rerank_k = 20) const;

    /**
     * Query + HSV color histogram re-scoring on FAISS candidates.
     * Best for cross-floor disambiguation. Does NOT use geometric re-ranking.
     * Requires `color_rerank=true` in the constructor.
     */
    QueryResult queryWithColorRerank(const std::string& image_path,
                                     int top_k       = 5,
                                     int candidates_k = 20) const;

    // ----------------------------------------------------------------
    // Persistence
    // ----------------------------------------------------------------

    /// Save the database (FAISS index + descriptors + metadata) to a directory.
    void saveDatabase(const std::string& directory) const;

    /// Load a previously saved database. Returns the number of images.
    int loadDatabase(const std::string& directory);

    // ----------------------------------------------------------------
    // Descriptor access
    // ----------------------------------------------------------------

    /**
     * Extract a global descriptor for a single image file.
     * @param[out] out  Pointer to pre-allocated float array of size
     *                  descriptorDim().
     */
    void extractDescriptor(const std::string& image_path, float* out) const;

    /**
     * Extract a global descriptor from a raw image buffer.
     * @param[out] out  Pointer to pre-allocated float array.
     */
    void extractDescriptorFromImage(const uint8_t* data,
                                    int height, int width,
                                    float* out) const;

    /// Descriptor dimensionality (8448 for both MegaLoc and SALAD-indoor).
    int descriptorDim() const;

private:
    std::unique_ptr<VPREngineImpl> impl_;
};

}  // namespace vrr
