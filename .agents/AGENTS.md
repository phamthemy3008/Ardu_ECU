# Version Bumping Rule
Chỉ tăng version (bumping version) khi người dùng yêu cầu thực hiện `git commit`, `git push` hoặc tạo release. Không tự động tăng version khi chỉnh sửa file lẻ trong quá trình phát triển.

# Git Commit & Push Rule
DO NOT automatically run `git commit` or `git push` unless the user explicitly requests it (e.g., "commit", "push", "tạo release", etc.). Always present code changes to the user and wait for their explicit directive before running git commit or git push commands.
