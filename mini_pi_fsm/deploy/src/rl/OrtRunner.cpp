// rl::OrtRunner -- ONNX Runtime inference backend.
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
//   - every output element is checked for finiteness on every inference.
#include "rl/ModelRunner.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#if defined(MINI_PI_WITH_ONNXRUNTIME)
#include <onnxruntime_cxx_api.h>
#endif

namespace mini_pi
{
namespace rl
{

bool haveInferenceBackend()
{
#if defined(MINI_PI_WITH_ONNXRUNTIME)
    return true;
#else
    return false;
#endif
}

#if defined(MINI_PI_WITH_ONNXRUNTIME)
namespace
{

class OrtRunner : public ModelRunner
{
public:
    explicit OrtRunner(const std::string& model_path)
        : env_(ORT_LOGGING_LEVEL_WARNING, "mini_pi_policy"), path_(model_path)
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

            auto shape = session_->GetInputTypeInfo(i)
                             .GetTensorTypeAndShapeInfo().GetShape();
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
        auto oname = session_->GetOutputNameAllocated(0, allocator_);
        output_name_ = oname.get();

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

    const std::vector<float>& act(const std::map<std::string, std::vector<float>>& obs) override
    {
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

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
                mem, buffers_[i].data(), buffers_[i].size(),
                input_shapes_[i].data(), input_shapes_[i].size()));
        }

        auto out = session_->Run(Ort::RunOptions{nullptr}, input_name_ptrs_.data(),
                                 tensors.data(), tensors.size(),
                                 output_name_ptrs_.data(), 1);

        const float* p = out.front().GetTensorMutableData<float>();
        std::memcpy(action_.data(), p, output_size_ * sizeof(float));

        for (std::size_t i = 0; i < action_.size(); ++i)
        {
            if (!std::isfinite(action_[i]))
                throw std::runtime_error("model output[" + std::to_string(i) +
                                         "] is not finite");
        }
        return action_;
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
    Ort::AllocatorWithDefaultOptions allocator_;

    std::string path_;
    std::vector<std::string> input_names_;
    std::vector<const char*> input_name_ptrs_;
    std::string output_name_;
    std::vector<const char*> output_name_ptrs_;

    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::size_t> input_sizes_;
    std::vector<int64_t> output_shape_;
    std::size_t output_size_ = 0;

    std::vector<std::vector<float>> buffers_;
    std::vector<float> action_;
};

} // namespace
#endif // MINI_PI_WITH_ONNXRUNTIME

std::unique_ptr<ModelRunner> createOrtRunner(const std::string& model_path, std::string& error)
{
#if !defined(MINI_PI_WITH_ONNXRUNTIME)
    (void)model_path;
    error = "this build has no inference backend (MINI_PI_ENABLE_RL=OFF or ONNX Runtime "
            "was not found at configure time -- see deploy/CMakeLists.txt)";
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

} // namespace rl
} // namespace mini_pi
