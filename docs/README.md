# Documentation

This folder contains operational documentation for the ML Yacht Autopilot project.

## Guides

| Document | Description |
|----------|-------------|
| [Data Preparation](data_preparation.md) | Complete guide for preprocessing logs and preparing training data |

## Quick Reference

### Log Organization

```bash
# Preview what will be organized
uv run python -m src.n2k.log_organizer n2klogs/raw/

# Execute organization
uv run python -m src.n2k.log_organizer n2klogs/raw/ --execute
```

### Log Analysis

```bash
# Analyze logs and generate metadata
uv run python -m src.training.log_analyzer n2klogs/raw/

# Dry run (preview only)
uv run python -m src.training.log_analyzer n2klogs/raw/ --dry-run
```

## Additional Resources

- [planning/](../planning/) - Design documents and implementation plans
- [specification.md](../specification.md) - System specification
- [README.md](../README.md) - Project overview
