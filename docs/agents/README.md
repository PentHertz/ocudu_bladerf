# Agent reference documentation

Reference material written for AI coding agents working on OCUDU, rather than for people. Pages here describe
conventions, architecture and workflows an agent needs to produce changes that fit the project.

The developer guidelines are documented in full at <https://docs.ocudu.org/dev_guide/>. Pages here must stay aligned
with that guide and carry only what an agent cannot derive from the sources and gets wrong in practice. A page earns its
place by fixing a mistake agents actually repeat, not by restating a rule they already follow.

Each `AGENTS.md` file has a `CLAUDE.md` file next to it that imports it. These files are at the repository root and in
some subdirectories, for example `apps/` and `lib/scheduler/`. They stay where their tooling expects them.

Do not copy the rules of a page of this directory into an `AGENTS.md` file. Add a link to the page in the `AGENTS.md`
file. For example, the root `AGENTS.md` file has the line
`Follow [docs/agents/code_comments.md](docs/agents/code_comments.md).`

Put shared instructions in `AGENTS.md`, not in `CLAUDE.md`. Put personal or Claude-specific instructions in a
`CLAUDE.local.md` file, which git ignores.

## Guidelines

- One topic per file, named after the topic.
- Document a convention or invariant once agents have tripped on it, not in anticipation.
- Keep pages current. A stale page misleads an agent more than a missing one.
