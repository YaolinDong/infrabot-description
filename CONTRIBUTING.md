# Contributing

When reporting a problem, include the model version, the link or joint involved, and the command that reproduces it.

## Checks

Requires the packages in `requirements.txt`.

```bash
python -B tools/validate_model.py
python -B -m unittest discover -s tests -v
```

The validator compares the URDF, meshes and friction file with the parameters and hashes in `model_manifest.json`. If you change the model on purpose, update the manifest in the same commit and explain the change.

## Preview image

```bash
python -m pip install -r tools/requirements-preview.txt
python -B tools/render_preview.py --output docs/images/h7c.png
```

## Commits

Use `type(scope): description` in English, for example `fix(urdf): correct the flange mesh path`.
