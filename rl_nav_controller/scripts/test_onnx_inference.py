#!/usr/bin/env python3
"""
Test script to verify ONNX model inference works correctly.

This script tests the ONNX models without requiring ROS dependencies,
making it useful for verifying the conversion and inference pipeline.

Model Architecture:
    VAE Encoder:
        - Input: depth_image [batch, 1, 40, 64]
        - Output: mu [batch, 64, 5, 8] -> flattened to 2560

    Navigation Policy (LSTM-based):
        - Inputs:
            - obs: [batch, 2576] (16 state features + 2560 depth embedding)
            - h_in: [1, batch, 512] (LSTM hidden state)
            - c_in: [1, batch, 512] (LSTM cell state)
        - Outputs:
            - actions: [batch, 3]
            - h_out: [1, batch, 512]
            - c_out: [1, batch, 512]

Usage:
    python test_onnx_inference.py --model-dir /path/to/onnx/models
"""

import argparse
import os
import time
import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("Error: onnxruntime is required.")
    print("Install with: pip install onnxruntime")
    exit(1)

try:
    from scipy.ndimage import zoom
except ImportError:
    print("Error: scipy is required.")
    print("Install with: pip install scipy")
    exit(1)


# Model dimensions
STATE_DIM = 16  # linear_vel(3) + angular_vel(3) + gravity(3) + last_action(3) + target_pos_log(4)
DEPTH_EMBEDDING_DIM = 2560  # 64 * 5 * 8 (flattened VAE output)
POLICY_INPUT_DIM = STATE_DIM + DEPTH_EMBEDDING_DIM  # 2576
LSTM_HIDDEN_DIM = 512
ACTION_DIM = 3


def test_vae_encoder(model_path: str, num_runs: int = 100) -> bool:
    """Test VAE encoder inference.

    Args:
        model_path: Path to VAE encoder ONNX model
        num_runs: Number of inference runs for timing

    Returns:
        True if test passed, False otherwise
    """
    print(f"\nTesting VAE encoder: {model_path}")

    if not os.path.exists(model_path):
        print(f"  ERROR: Model file not found")
        return False

    try:
        # Load model
        session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        # Get input/output info
        input_info = session.get_inputs()[0]
        output_info = session.get_outputs()[0]
        print(f"  Input:  {input_info.name} - shape: {input_info.shape}, type: {input_info.type}")
        print(f"  Output: {output_info.name} - shape: {output_info.shape}, type: {output_info.type}")

        # Create dummy input (simulating 600x960 depth image resized to 40x64)
        dummy_depth = np.random.rand(600, 960).astype(np.float32) * 10.0  # 0-10m depth
        zoom_factors = (40 / 600, 64 / 960)
        resized_depth = zoom(dummy_depth, zoom_factors, order=1)
        input_tensor = resized_depth[np.newaxis, np.newaxis, :, :].astype(np.float32)

        print(f"  Input tensor shape: {input_tensor.shape}")

        # Run inference
        output = session.run(None, {input_info.name: input_tensor})[0]
        print(f"  Output shape: {output.shape}")

        # Flatten the output as used in the actual controller
        flattened = output.flatten()
        print(f"  Flattened embedding shape: {flattened.shape}")
        print(f"  Output range: [{output.min():.4f}, {output.max():.4f}]")

        # Verify expected dimensions
        expected_flat_dim = DEPTH_EMBEDDING_DIM
        if flattened.shape[0] == expected_flat_dim:
            print(f"  Embedding dimension check: PASSED ({expected_flat_dim})")
        else:
            print(f"  WARNING: Expected embedding dim {expected_flat_dim}, got {flattened.shape[0]}")

        # Timing test
        start = time.time()
        for _ in range(num_runs):
            session.run(None, {input_info.name: input_tensor})
        elapsed = time.time() - start
        avg_time = (elapsed / num_runs) * 1000
        print(f"  Average inference time: {avg_time:.2f} ms ({num_runs} runs)")

        print("  TEST PASSED")
        return True

    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def test_nav_policy(model_path: str, num_runs: int = 100) -> bool:
    """Test navigation policy inference (LSTM-based).

    Args:
        model_path: Path to navigation policy ONNX model
        num_runs: Number of inference runs for timing

    Returns:
        True if test passed, False otherwise
    """
    print(f"\nTesting navigation policy: {model_path}")

    if not os.path.exists(model_path):
        print(f"  ERROR: Model file not found")
        return False

    try:
        # Load model
        session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        # Get input/output info
        print("  Model inputs:")
        for inp in session.get_inputs():
            print(f"    - {inp.name}: shape {inp.shape}, type {inp.type}")
        print("  Model outputs:")
        for out in session.get_outputs():
            print(f"    - {out.name}: shape {out.shape}, type {out.type}")

        # Create dummy inputs matching the LSTM policy architecture
        batch_size = 1
        obs = np.random.randn(batch_size, POLICY_INPUT_DIM).astype(np.float32)
        h_in = np.zeros((1, batch_size, LSTM_HIDDEN_DIM), dtype=np.float32)
        c_in = np.zeros((1, batch_size, LSTM_HIDDEN_DIM), dtype=np.float32)

        print(f"\n  Test inputs:")
        print(f"    obs shape: {obs.shape}")
        print(f"    h_in shape: {h_in.shape}")
        print(f"    c_in shape: {c_in.shape}")

        # Run inference
        outputs = session.run(
            None,
            {'obs': obs, 'h_in': h_in, 'c_in': c_in}
        )
        actions, h_out, c_out = outputs

        print(f"\n  Test outputs:")
        print(f"    actions shape: {actions.shape}")
        print(f"    h_out shape: {h_out.shape}")
        print(f"    c_out shape: {c_out.shape}")
        print(f"    Raw action: {actions}")

        # Apply tanh and scaling (as done in the actual code)
        policy_scale = np.array([1.5, 1.0, 1.0], dtype=np.float32)
        cmd_vel = np.tanh(actions) * policy_scale
        print(f"    Scaled cmd_vel: {cmd_vel}")

        # Test sequential inference (simulating time steps)
        print(f"\n  Testing sequential inference (3 steps):")
        h, c = h_in.copy(), c_in.copy()
        for step in range(3):
            outputs = session.run(None, {'obs': obs, 'h_in': h, 'c_in': c})
            actions, h, c = outputs
            scaled = np.tanh(actions) * policy_scale
            print(f"    Step {step+1}: action={scaled.flatten()}")

        # Timing test
        h, c = h_in.copy(), c_in.copy()
        start = time.time()
        for _ in range(num_runs):
            outputs = session.run(None, {'obs': obs, 'h_in': h, 'c_in': c})
            _, h, c = outputs
        elapsed = time.time() - start
        avg_time = (elapsed / num_runs) * 1000
        print(f"\n  Average inference time: {avg_time:.2f} ms ({num_runs} runs)")

        print("  TEST PASSED")
        return True

    except Exception as e:
        print(f"  ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_full_pipeline(vae_path: str, policy_path: str) -> bool:
    """Test the full inference pipeline (VAE + LSTM Policy).

    Args:
        vae_path: Path to VAE encoder ONNX model
        policy_path: Path to navigation policy ONNX model

    Returns:
        True if test passed, False otherwise
    """
    print(f"\nTesting full inference pipeline:")

    if not os.path.exists(vae_path) or not os.path.exists(policy_path):
        print(f"  ERROR: One or both model files not found")
        return False

    try:
        # Load models
        vae_session = ort.InferenceSession(vae_path, providers=['CPUExecutionProvider'])
        policy_session = ort.InferenceSession(policy_path, providers=['CPUExecutionProvider'])

        # Simulate real inputs
        depth_image = np.random.rand(600, 960).astype(np.float32) * 10.0
        linear_vel = [0.1, 0.0, 0.0]
        angular_vel = [0.0, 0.0, 0.1]
        gravity_vector = [0.0, 0.0, -1.0]
        last_action = [0.0, 0.0, 0.0]
        target_pos_log = [1.0, 0.0, 0.0, 0.5]  # normalized direction + log distance

        # Preprocess depth image
        zoom_factors = (40 / 600, 64 / 960)
        resized_depth = zoom(depth_image, zoom_factors, order=1)
        depth_tensor = resized_depth[np.newaxis, np.newaxis, :, :].astype(np.float32)

        # Run VAE encoder
        start = time.time()
        vae_output = vae_session.run(
            None,
            {vae_session.get_inputs()[0].name: depth_tensor}
        )[0]
        embedding = vae_output.flatten()
        vae_time = (time.time() - start) * 1000
        print(f"  VAE encoder output shape: {vae_output.shape} -> flattened: {embedding.shape}")
        print(f"  VAE inference time: {vae_time:.2f} ms")

        # Assemble policy input
        # State: linear_vel(3) + angular_vel(3) + gravity(3) + last_action(3) + target_pos_log(4) = 16
        state_input = np.array(
            linear_vel + angular_vel + gravity_vector + last_action + target_pos_log,
            dtype=np.float32
        )
        obs = np.concatenate([state_input, embedding])[np.newaxis].astype(np.float32)

        # Initialize LSTM states
        batch_size = 1
        h_in = np.zeros((1, batch_size, LSTM_HIDDEN_DIM), dtype=np.float32)
        c_in = np.zeros((1, batch_size, LSTM_HIDDEN_DIM), dtype=np.float32)

        print(f"  Policy input (obs) shape: {obs.shape}")
        print(f"    - State features: {STATE_DIM}")
        print(f"    - Depth embedding: {embedding.shape[0]}")
        print(f"  LSTM hidden states shape: {h_in.shape}")

        # Run policy
        start = time.time()
        outputs = policy_session.run(
            None,
            {'obs': obs, 'h_in': h_in, 'c_in': c_in}
        )
        raw_action, h_out, c_out = outputs
        policy_time = (time.time() - start) * 1000
        print(f"  Policy output shape: {raw_action.shape}, time: {policy_time:.2f} ms")

        # Apply tanh and scaling
        policy_scale = np.array([1.5, 1.0, 1.0], dtype=np.float32)
        cmd_vel = np.tanh(raw_action) * policy_scale
        print(f"  Final cmd_vel: linear_x={cmd_vel[0,0]:.4f}, linear_y={cmd_vel[0,1]:.4f}, angular_z={cmd_vel[0,2]:.4f}")

        total_time = vae_time + policy_time
        print(f"\n  Total inference time: {total_time:.2f} ms")
        print(f"  Max frequency: {1000/total_time:.1f} Hz")

        # Test multiple sequential steps
        print(f"\n  Testing 5 sequential inference steps:")
        h, c = h_out, c_out
        for step in range(5):
            # Simulate new depth image
            new_depth = np.random.rand(600, 960).astype(np.float32) * 10.0
            resized = zoom(new_depth, zoom_factors, order=1)
            depth_tensor = resized[np.newaxis, np.newaxis, :, :].astype(np.float32)

            # VAE
            vae_out = vae_session.run(None, {vae_session.get_inputs()[0].name: depth_tensor})[0]
            emb = vae_out.flatten()

            # Policy
            obs = np.concatenate([state_input, emb])[np.newaxis].astype(np.float32)
            outputs = policy_session.run(None, {'obs': obs, 'h_in': h, 'c_in': c})
            action, h, c = outputs
            scaled = np.tanh(action) * policy_scale
            print(f"    Step {step+1}: cmd_vel = [{scaled[0,0]:.3f}, {scaled[0,1]:.3f}, {scaled[0,2]:.3f}]")

        print("\n  FULL PIPELINE TEST PASSED")
        return True

    except Exception as e:
        print(f"  ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Test ONNX model inference'
    )
    parser.add_argument(
        '--model-dir',
        type=str,
        default='deployment_policies',
        help='Directory containing ONNX models'
    )
    parser.add_argument(
        '--vae-model',
        type=str,
        default='vae_encoder.onnx',
        help='Filename of VAE encoder model'
    )
    parser.add_argument(
        '--policy-model',
        type=str,
        default='nav_policy.onnx',
        help='Filename of navigation policy model'
    )
    parser.add_argument(
        '--num-runs',
        type=int,
        default=100,
        help='Number of runs for timing tests'
    )

    args = parser.parse_args()

    vae_path = os.path.join(args.model_dir, args.vae_model)
    policy_path = os.path.join(args.model_dir, args.policy_model)

    print("=" * 60)
    print("ONNX Inference Test")
    print("=" * 60)
    print(f"Model directory: {args.model_dir}")
    print(f"VAE model:       {args.vae_model}")
    print(f"Policy model:    {args.policy_model}")
    print(f"\nExpected dimensions:")
    print(f"  VAE input:     [batch, 1, 40, 64]")
    print(f"  VAE output:    [batch, 64, 5, 8] -> flattened: {DEPTH_EMBEDDING_DIM}")
    print(f"  Policy input:  [batch, {POLICY_INPUT_DIM}] ({STATE_DIM} state + {DEPTH_EMBEDDING_DIM} embedding)")
    print(f"  LSTM hidden:   [1, batch, {LSTM_HIDDEN_DIM}]")
    print(f"  Policy output: [batch, {ACTION_DIM}]")

    # Check available providers
    print(f"\nAvailable ONNX Runtime providers: {ort.get_available_providers()}")

    all_passed = True

    # Test individual models
    if not test_vae_encoder(vae_path, args.num_runs):
        all_passed = False

    if not test_nav_policy(policy_path, args.num_runs):
        all_passed = False

    # Test full pipeline
    if os.path.exists(vae_path) and os.path.exists(policy_path):
        if not test_full_pipeline(vae_path, policy_path):
            all_passed = False

    print("\n" + "=" * 60)
    if all_passed:
        print("ALL TESTS PASSED!")
    else:
        print("SOME TESTS FAILED - check output above")
    print("=" * 60)


if __name__ == '__main__':
    main()
