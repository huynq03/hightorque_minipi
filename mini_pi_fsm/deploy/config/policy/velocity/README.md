# Production policy packages -- `Velocity`

The real trained Mini-Pi policy is installed as `v0`. Production simulation
therefore resolves it normally without the TEST-policy launch override.

## Package layout

```
config/policy/velocity/
└── v0/
    ├── params/deploy.yaml
    └── exported/policy.onnx
```

Both files are required. A directory missing either one is rejected with a
message naming the missing file; it is not silently skipped.

The version directory name is free-form. When `policy_dir` points at this
directory (the normal case), versions are sorted **lexicographically** and the
last valid one wins — so `v0 < v1 < v2`, but `v10 < v9`. Zero-pad past `v9`.
Pin a specific one with `policy.version` in `../../policy.yaml`.

The selected directory is printed at startup:

```
PolicyRunner: selected policy package '<abs path>' (version 'v0')
```

## What `deploy.yaml` must contain

See `../../../test/fixtures/policy/velocity/v0/params/deploy.yaml` for a fully
commented example, and `../../../docs/rl_architecture.md` for the schema.
Required keys: `step_dt`, `joint_names`, `default_joint_pos`, `stiffness`,
`damping`, `observations`, `actions`.

A real policy must **not** set `simulation_only: true`. That flag remains
reserved for deterministic test fixtures and is refused on non-simulation
hardware backends.

## Before the first physical run

This package has been validated in simulation only. Physical RL deployment is
a separate safety phase; installing this package does not authorize it.
