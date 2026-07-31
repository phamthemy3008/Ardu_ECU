# Version Bumping Rule
Whenever you make a modification to `index.html` or `ECU_ManualV1.ino`, you MUST increment the version number by 0.1 (e.g. from 1.2 to 1.3) so the user can visually confirm the update on the Web Dashboard.

Locations to update:
1. `index.html`: Update the `Web UI vX.X` text in the `<h2>` title (around line 213).
2. `ECU_ManualV1.ino`: Update `Serial.print(" | VER=X.X");` inside `sendWebStatus()`.
3. `ECU_ManualV1.ino`: Update the boot log `VERSION X.X` inside `setup()`.

# Git Commit & Push Rule
DO NOT automatically run `git commit` or `git push` unless the user explicitly requests it (e.g., "commit", "push", "tạo release", etc.). Always present code changes to the user and wait for their explicit directive before running git commit or git push commands.
