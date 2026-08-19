import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const rootDir = path.resolve(__dirname, '..');

const versionJsonPath = path.join(rootDir, 'version.json');
const packageJsonPath = path.join(rootDir, 'package.json');
const indexHtmlPath = path.join(rootDir, 'PROJECT_CURRENT', 'Tools', 'index.html');
const inoPath = path.join(rootDir, 'PROJECT_CURRENT', 'Firmware', 'ECU_ManualV1', 'ECU_ManualV1.ino');
const changelogPath = path.join(rootDir, 'PROJECT_CURRENT', 'Docs', 'CHANGELOG.md');

function run() {
  const args = process.argv.slice(2);
  const isCheckOnly = args.includes('--check');
  const setIndex = args.indexOf('--set');
  
  if (!fs.existsSync(versionJsonPath)) {
    console.error('Error: version.json not found!');
    process.exit(1);
  }

  const versionData = JSON.parse(fs.readFileSync(versionJsonPath, 'utf8'));
  const currentVersionStr = versionData.version || '6.4';
  const currentVersionNum = parseFloat(currentVersionStr);

  let newVersionNum = currentVersionNum;
  if (setIndex !== -1 && args[setIndex + 1]) {
    newVersionNum = parseFloat(args[setIndex + 1]);
  } else if (!isCheckOnly) {
    newVersionNum = Math.round((currentVersionNum + 0.1) * 10) / 10;
  }

  const newVersionStr = newVersionNum.toFixed(1);
  console.log(`Version transition: ${currentVersionStr} -> ${newVersionStr}`);

  if (isCheckOnly) {
    console.log(`Current version is v${currentVersionStr}`);
    return;
  }

  const nowIso = new Date().toISOString();
  const dateStr = nowIso.split('T')[0];

  // 1. Update version.json
  versionData.version = newVersionStr;
  versionData.updatedAt = nowIso;
  fs.writeFileSync(versionJsonPath, JSON.stringify(versionData, null, 2) + '\n', 'utf8');
  console.log(`[✓] Updated version.json -> v${newVersionStr}`);

  // 2. Update package.json
  if (fs.existsSync(packageJsonPath)) {
    const pkg = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
    pkg.version = `${newVersionStr}.0`;
    fs.writeFileSync(packageJsonPath, JSON.stringify(pkg, null, 2) + '\n', 'utf8');
    console.log(`[✓] Updated package.json -> v${pkg.version}`);
  }

  // 3. Update index.html
  if (fs.existsSync(indexHtmlPath)) {
    let content = fs.readFileSync(indexHtmlPath, 'utf8');
    // Pattern: Web UI vX.X
    content = content.replace(/Web UI v\d+\.\d+/g, `Web UI v${newVersionStr}`);
    fs.writeFileSync(indexHtmlPath, content, 'utf8');
    console.log(`[✓] Updated index.html -> Web UI v${newVersionStr}`);
  }

  // 4. Update ECU_ManualV1.ino
  if (fs.existsSync(inoPath)) {
    let content = fs.readFileSync(inoPath, 'utf8');
    content = content.replace(/VER=\d+\.\d+/g, `VER=${newVersionStr}`);
    content = content.replace(/VERSION \d+\.\d+/g, `VERSION ${newVersionStr}`);
    fs.writeFileSync(inoPath, content, 'utf8');
    console.log(`[✓] Updated ECU_ManualV1.ino -> VER=${newVersionStr} / VERSION ${newVersionStr}`);
  }

  // 5. Update CHANGELOG.md
  if (fs.existsSync(changelogPath)) {
    let changelog = fs.readFileSync(changelogPath, 'utf8');
    if (!changelog.includes(`## [v${newVersionStr}]`)) {
      const newEntry = `## [v${newVersionStr}] - ${dateStr}\n- **CI/CD Auto Release**: Automatically incremented version to v${newVersionStr}.\n- Synchronized Web UI v${newVersionStr} and Firmware ECU_ManualV1.ino (VER=${newVersionStr}).\n\n`;
      // Insert right after title
      if (changelog.includes('# Ardu ECU Manual V1 - Changelog & Version History\n\n')) {
        changelog = changelog.replace('# Ardu ECU Manual V1 - Changelog & Version History\n\n', `# Ardu ECU Manual V1 - Changelog & Version History\n\n${newEntry}`);
      } else {
        changelog = `${newEntry}${changelog}`;
      }
      fs.writeFileSync(changelogPath, changelog, 'utf8');
      console.log(`[✓] Updated CHANGELOG.md -> Added section [v${newVersionStr}]`);
    }
  }

  // 6. Update README.md
  const readmePath = path.join(rootDir, 'PROJECT_CURRENT', 'README.md');
  if (fs.existsSync(readmePath)) {
    let readme = fs.readFileSync(readmePath, 'utf8');
    readme = readme.replace(/\(Version \d+\.\d+\)/g, `(Version ${newVersionStr})`);
    readme = readme.replace(/Phiên bản v\d+\.\d+/g, `Phiên bản v${newVersionStr}`);
    readme = readme.replace(/Cập nhật\*\*: \d{4}-\d{2}-\d{2}/g, `Cập nhật**: ${dateStr}`);
    fs.writeFileSync(readmePath, readme, 'utf8');
    console.log(`[✓] Updated README.md -> Version ${newVersionStr}`);
  }

  console.log(`Successfully bumped project version to v${newVersionStr}`);
}

run();
