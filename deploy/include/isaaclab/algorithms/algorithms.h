// Algorithms / OrtRunner -- the policy network behind one interface.
//
// Counterpart of unitree_rl_mjlab/deploy/include/isaaclab/algorithms/
// algorithms.h: one flattened observation per ONNX input name goes in, the raw
// network output comes out. It knows nothing about joints, scaling or history.
// `Algorithms` is also the seam tests use to substitute a deterministic model;
// createOrtRunner() is the only production implementation.
//
// NOTHING HERE KNOWS A POLICY OR A DIMENSION. This interface only REPORTS what
// the model declares; ManagerBasedRLEnv::set_algorithm() checks that against
// the ObservationManager groups and the ActionManager width.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace isaaclab
{

/// One model input or output exactly as the model DECLARES it -- before the
/// runner substitutes 1 for a dynamic batch axis. Lets a contract check tell a
/// fixed dimension from a dynamic one, and float32 from anything else.
struct TensorSpec
{
    std::string name;
    /// Declared dims; -1 marks a dynamic axis.
    std::vector<std::int64_t> shape;
    bool float32 = true;
    /// Element type for messages ("float32", "float64", ...).
    std::string type = "float32";
};

class Algorithms
{
public:
    virtual ~Algorithms() = default;

    // ---- declared contract ----------------------------------------------
    // Defaults describe the plain [1, N] float32 single-output model every
    // test double stands in for; OrtRunner reports what the file declares.
    virtual std::size_t outputCount() const { return 1; }
    virtual TensorSpec inputSpec(std::size_t i) const
    {
        return {inputNames().at(i), {1, static_cast<std::int64_t>(inputSize(i))}, true, "float32"};
    }
    virtual TensorSpec outputSpec(std::size_t) const
    {
        return {"output", {1, static_cast<std::int64_t>(outputSize())}, true, "float32"};
    }

    /// Input tensor names, in the model's own order. An observation group of
    /// the same name supplies each one.
    virtual const std::vector<std::string>& inputNames() const = 0;
    /// Flattened element count the model expects for input `i`.
    virtual std::size_t inputSize(std::size_t i) const = 0;
    /// Flattened element count of output 0.
    virtual std::size_t outputSize() const = 0;

    /// Runs one inference. Throws std::runtime_error if an input is missing or
    /// has the wrong size. ManagerBasedRLEnv::step() rejects non-finite output.
    ///
    /// The returned reference points at an internal buffer that the NEXT call
    /// overwrites. Copy it if it has to outlive the next inference -- comparing
    /// two such references compares a value with itself.
    virtual const std::vector<float>& act(
        const std::map<std::string, std::vector<float>>& obs) = 0;

    virtual std::string describe() const = 0;
};

/// Generic contract check of one declared tensor against the number of values
/// the policy configuration produces (input) or consumes (output):
///   - float32;
///   - leading axis is the batch: 1 or dynamic (run as 1);
///   - every other axis FIXED -- a dynamic feature axis is refused;
///   - the fixed axes multiply to `expected`.
/// `what` is "input" or "output". Empty string when it fits, else the reason.
std::string checkTensorContract(const TensorSpec& t, const char* what, std::size_t expected);

/// Creates the ONNX Runtime backend for `model_path`.
///
/// Returns nullptr and fills `error` when the file is missing, is not a
/// loadable model, or when this build has no inference backend compiled in
/// (ISAACLAB_WITH_ONNXRUNTIME, defined by the robot package's build, is unset).
/// Never throws.
std::unique_ptr<Algorithms> createOrtRunner(const std::string& model_path,
                                             std::string& error);


/// False when this build was configured without an inference backend.
bool haveInferenceBackend();

} // namespace isaaclab
