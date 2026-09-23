// rl::ModelRunner -- the inference backend interface.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/
// algorithms/algorithms.h (`Algorithms` / `OrtRunner`).
//
// NOTHING HERE KNOWS 705 OR 12. The expected dimensions belong to the policy
// package and to the model file; this interface only REPORTS what the model
// declares, and PolicyEnvironment cross-checks that against the configured
// observation and action widths. A generic runner with a baked-in dimension
// would silently accept the wrong model.
#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mini_pi
{
namespace rl
{

class ModelRunner
{
public:
    virtual ~ModelRunner() = default;

    /// Input tensor names, in the model's own order. An observation group of
    /// the same name supplies each one.
    virtual const std::vector<std::string>& inputNames() const = 0;
    /// Flattened element count the model expects for input `i`.
    virtual std::size_t inputSize(std::size_t i) const = 0;
    /// Flattened element count of output 0.
    virtual std::size_t outputSize() const = 0;

    /// Runs one inference. Throws std::runtime_error if an input is missing,
    /// has the wrong size, or if any output is not finite.
    ///
    /// The returned reference points at an internal buffer that the NEXT call
    /// overwrites. Copy it if it has to outlive the next inference -- comparing
    /// two such references compares a value with itself.
    virtual const std::vector<float>& act(
        const std::map<std::string, std::vector<float>>& obs) = 0;

    virtual std::string describe() const = 0;
};

/// Creates the ONNX Runtime backend for `model_path`.
///
/// Returns nullptr and fills `error` when the file is missing, is not a
/// loadable model, or when this build has no inference backend compiled in
/// (MINI_PI_WITH_ONNXRUNTIME undefined -- see deploy/CMakeLists.txt).
/// Never throws.
std::unique_ptr<ModelRunner> createOrtRunner(const std::string& model_path,
                                             std::string& error);

/// False when this build was configured without an inference backend.
bool haveInferenceBackend();

} // namespace rl
} // namespace mini_pi
