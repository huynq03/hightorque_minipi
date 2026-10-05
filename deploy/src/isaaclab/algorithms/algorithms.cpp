// OrtRunner -- ONNX Runtime inference backend.
//
// Adapted from unitree_rl_mjlab/deploy/include/isaaclab/algorithms/algorithms.h
// `OrtRunner`. Kept: the session setup, reading input names and shapes from the
// session, requiring every input name to be present in the observation map,
// and copying output 0.
//
// Added, all because a wrong-shaped or NaN-producing model must fail loudly
// rather than drive the robot:
//   - a dynamic (-1) batch dimension is treated as 1 instead of multiplying
//     the element count by -1;
//   - the supplied observation length is checked against the model's declared
//     input size, per input;
//   - every output element is checked for finiteness on every inference;
//   - the element count the session actually returned is checked against the
//     declared output size BEFORE it is copied;
//   - the declared shapes and element types are kept as declared (dynamic
//     axes as -1) for contract checks: see TensorSpec.
#include "isaaclab/algorithms/algorithms.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#if defined(ISAACLAB_WITH_ONNXRUNTIME)
#include <onnxruntime_cxx_api.h>
#endif

namespace isaaclab
{

bool haveInferenceBackend()
{
#if defined(ISAACLAB_WITH_ONNXRUNTIME)
    return true;
#else
    return false;
#endif
}

#if defined(ISAACLAB_WITH_ONNXRUNTIME)
namespace
{

std::string elementTypeName(ONNXTensorElementDataType t)
{
    switch (t)
    {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:   return "float32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:  return "float64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: return "float16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:   return "int32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:   return "int64";
        default: return "element type " + std::to_string(static_cast<int>(t));
    }
}

TensorSpec specOf(const std::string& name, const Ort::TypeInfo& info)
{
    TensorSpec s;
    s.name = name;
    if (info.GetONNXType() != ONNX_TYPE_TENSOR)
    {
        s.float32 = false;
        s.type = "non-tensor";
        return s;
    }
    const auto t = info.GetTensorTypeAndShapeInfo();
    s.shape = t.GetShape();
    s.float32 = t.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
    s.type = elementTypeName(t.GetElementType());
    return s;
}

class OrtRunner : public Algorithms
{
public:
    explicit OrtRunner(const std::string& model_path)
        : env_(ORT_LOGGING_LEVEL_WARNING, "isaaclab_policy"), path_(model_path)
    {
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        // One thread. The policy runs at 50 Hz on a 12-DOF MLP; a thread pool
        // buys nothing and introduces scheduling jitter next to a 1 kHz
        // control loop.
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetInterOpNumThreads(1);

        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

        const std::size_t n_in = session_->GetInputCount();
        if (n_in == 0) throw std::runtime_error("model has no inputs");

        for (std::size_t i = 0; i < n_in; ++i)
        {
            auto name = session_->GetInputNameAllocated(i, allocator_);
            input_names_.emplace_back(name.get());

            input_specs_.push_back(specOf(input_names_.back(), session_->GetInputTypeInfo(i)));
            auto shape = input_specs_.back().shape;
            std::size_t count = 1;
            for (auto& d : shape)
            {
                // A dynamic axis comes back as -1. Every axis but the batch is
                // fixed for a policy, so a free axis is the batch and is 1.
                if (d < 0) d = 1;
                count *= static_cast<std::size_t>(d);
            }
            input_shapes_.push_back(std::move(shape));
            input_sizes_.push_back(count);
        }

        if (session_->GetOutputCount() == 0) throw std::runtime_error("model has no outputs");
        for (std::size_t i = 0; i < session_->GetOutputCount(); ++i)
        {
            auto name = session_->GetOutputNameAllocated(i, allocator_);
            output_specs_.push_back(specOf(name.get(), session_->GetOutputTypeInfo(i)));
        }
        output_name_ = output_specs_.front().name;

        output_shape_ = session_->GetOutputTypeInfo(0)
                            .GetTensorTypeAndShapeInfo().GetShape();
        output_size_ = 1;
        for (auto& d : output_shape_)
        {
            if (d < 0) d = 1;
            output_size_ *= static_cast<std::size_t>(d);
        }
        if (output_size_ == 0) throw std::runtime_error("model output 0 is empty");
        action_.assign(output_size_, 0.0f);

        for (const auto& s : input_names_) input_name_ptrs_.push_back(s.c_str());
        output_name_ptrs_.push_back(output_name_.c_str());
    }

    const std::vector<std::string>& inputNames() const override { return input_names_; }
    std::size_t inputSize(std::size_t i) const override { return input_sizes_.at(i); }
    std::size_t outputSize() const override { return output_size_; }
    std::size_t outputCount() const override { return output_specs_.size(); }
    TensorSpec inputSpec(std::size_t i) const override { return input_specs_.at(i); }
    TensorSpec outputSpec(std::size_t i) const override { return output_specs_.at(i); }

    const std::vector<float>& act(const std::map<std::string, std::vector<float>>& obs) override
    {
        // Copy, because Ort::Value::CreateTensor takes a mutable pointer and
        // must not alias the caller's const map.
        buffers_.resize(input_names_.size());
        std::vector<Ort::Value> tensors;
        tensors.reserve(input_names_.size());

        for (std::size_t i = 0; i < input_names_.size(); ++i)
        {
            auto it = obs.find(input_names_[i]);
            if (it == obs.end())
                throw std::runtime_error("model input '" + input_names_[i] +
                                         "' has no matching observation group");
            if (it->second.size() != input_sizes_[i])
                throw std::runtime_error("model input '" + input_names_[i] + "' expects " +
                                         std::to_string(input_sizes_[i]) + " values but the "
                                         "observation supplied " +
                                         std::to_string(it->second.size()));
            buffers_[i] = it->second;
            tensors.push_back(Ort::Value::CreateTensor<float>(
                mem_, buffers_[i].data(), buffers_[i].size(),
                input_shapes_[i].data(), input_shapes_[i].size()));
        }

        auto out = session_->Run(Ort::RunOptions{nullptr}, input_name_ptrs_.data(),
                                 tensors.data(), tensors.size(),
                                 output_name_ptrs_.data(), 1);

        // What the session RETURNED, not what it declared: a model with a
        // symbolic output axis can legally return more or fewer values.
        const auto info = out.front().GetTensorTypeAndShapeInfo();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("model output '" + output_name_ + "' is " +
                                     elementTypeName(info.GetElementType()) + ", not float32");
        if (info.GetElementCount() != output_size_)
            throw std::runtime_error("model output '" + output_name_ + "' returned " +
                                     std::to_string(info.GetElementCount()) +
                                     " values, declared " + std::to_string(output_size_));
        const float* p = out.front().GetTensorMutableData<float>();
        std::memcpy(action_.data(), p, output_size_ * sizeof(float));
        return action_;  // finiteness: ManagerBasedRLEnv::step(), for every Algorithms
    }

    std::string describe() const override
    {
        std::ostringstream os;
        os << "  model: " << path_ << "\n";
        for (std::size_t i = 0; i < input_names_.size(); ++i)
        {
            os << "    input  '" << input_names_[i] << "' [";
            for (std::size_t d = 0; d < input_shapes_[i].size(); ++d)
                os << (d ? "," : "") << input_shapes_[i][d];
            os << "] = " << input_sizes_[i] << " values\n";
        }
        os << "    output '" << output_name_ << "' [";
        for (std::size_t d = 0; d < output_shape_.size(); ++d)
            os << (d ? "," : "") << output_shape_[d];
        os << "] = " << output_size_ << " values\n";
        return os.str();
    }

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo mem_ = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::AllocatorWithDefaultOptions allocator_;

    std::string path_;
    std::vector<std::string> input_names_;
    std::vector<const char*> input_name_ptrs_;
    std::string output_name_;
    std::vector<const char*> output_name_ptrs_;

    std::vector<TensorSpec> input_specs_;
    std::vector<TensorSpec> output_specs_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::size_t> input_sizes_;
    std::vector<int64_t> output_shape_;
    std::size_t output_size_ = 0;

    std::vector<std::vector<float>> buffers_;
    std::vector<float> action_;
};

} // namespace
#endif // ISAACLAB_WITH_ONNXRUNTIME

std::string checkTensorContract(const TensorSpec& t, const char* what, std::size_t expected)
{
    std::ostringstream shape;
    shape << "[";
    for (std::size_t i = 0; i < t.shape.size(); ++i)
        shape << (i ? "," : "") << (t.shape[i] < 0 ? std::string("dyn") : std::to_string(t.shape[i]));
    shape << "]";
    const std::string who = std::string("model ") + what + " '" + t.name + "' " + shape.str();

    if (!t.float32) return who + " is " + t.type + ", expected float32";
    if (t.shape.empty()) return who + " has no axes";
    // [features] alone, or [batch, features...] with batch 1 or dynamic.
    const std::size_t first = t.shape.size() == 1 ? 0 : 1;
    if (first == 1 && t.shape[0] > 1) return who + ": batch axis must be 1 or dynamic";
    std::size_t n = 1;
    for (std::size_t i = first; i < t.shape.size(); ++i)
    {
        if (t.shape[i] <= 0) return who + " declares a dynamic feature axis; it must be fixed";
        n *= static_cast<std::size_t>(t.shape[i]);
    }
    if (n != expected)
        return who + " holds " + std::to_string(n) + " values, the policy configuration " +
               (std::string(what) == "input" ? "produces " : "expects ") + std::to_string(expected);
    return {};
}

std::unique_ptr<Algorithms> createOrtRunner(const std::string& model_path, std::string& error)
{
#if !defined(ISAACLAB_WITH_ONNXRUNTIME)
    (void)model_path;
    error = "this build has no inference backend (built without ONNX Runtime: "
            "ISAACLAB_WITH_ONNXRUNTIME is not defined by the robot package's build)";
    return nullptr;
#else
    std::error_code ec;
    if (!std::filesystem::is_regular_file(model_path, ec))
    {
        error = "model file '" + model_path + "' does not exist";
        return nullptr;
    }
    try
    {
        return std::make_unique<OrtRunner>(model_path);
    }
    catch (const std::exception& e)
    {
        error = "cannot load '" + model_path + "': " + e.what();
        return nullptr;
    }
#endif
}


} // namespace isaaclab
