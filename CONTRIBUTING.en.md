# Contributing

[简体中文](CONTRIBUTING.md) | [English](CONTRIBUTING.en.md)

Read [Algorithm Design](docs/en/algorithm.md) and [Architecture](docs/en/architecture.md) before making changes. Keep the algorithm, viewer, and HTTP interface separate. Avoid dependencies on application servers, robot services, or machine-specific absolute paths.

For algorithm changes, describe the input, the previous behavior, the resulting behavior, and the geometric constraints involved. Prefer small, reproducible synthetic regression scenes. Keep recorded maps in `data/local/` and make tests that need them optional.

Before submitting changes, run:

```bash
npm test
npm run build
```

Add a focused regression and run `npm run benchmark` when changing curves, constraints, or parameter behavior. Run `npm run test:web` and inspect screenshots when changing interactions or 3D rendering.

Keep dependencies, virtual environments, build output, logs, and maps without redistribution permission out of commits. Compare validation results together with route metrics; a shorter or smoother curve alone is insufficient.

Update the data-format documentation when public fields or parameter meanings change. Chinese is the default documentation language. Maintain the English version, examples, and language links together with the Chinese version. The C++ port should reuse existing formats and validation cases, with explicit numerical tolerances.
