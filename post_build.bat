@echo off
setlocal

REM --- Determine Paths ---
REM Assume this batch script (post_build.bat) is in the Project Root.
set "PROJECT_ROOT=%~dp0"
REM Remove trailing backslash from PROJECT_ROOT if it exists
if "%PROJECT_ROOT:~-1%"=="\" set "PROJECT_ROOT=%PROJECT_ROOT:~0,-1%"

REM Keil usually sets the CWD to the output directory (e.g., MDK-ARM\Objects) when running user commands.
REM Let's capture the Current Working Directory.
set "KEIL_CWD=%cd%"

REM --- Configuration ---
REM Set the FULL ABSOLUTE path to your Bootloader HEX file
set "BOOTLOADER_HEX_PATH=D:\project\pump_application_project\mm32\Bootloader\MDK-ARM\Objects\bootloader.hex"

REM Define the Application HEX filename (as per your setting)
set "APP_HEX_FILENAME=Objects\app.hex"

REM Define the Merged HEX filename
set "MERGED_HEX_FILENAME=Objects\boot_app.hex"

REM --- Path Constructions ---
REM Python script is in the project root
set "PYTHON_SCRIPT_PATH=%PROJECT_ROOT%\merge_hex.py"

REM Application HEX is expected to be in Keil's CWD (which should be the output folder)
set "APP_HEX_PATH=%KEIL_CWD%\%APP_HEX_FILENAME%"

REM Output merged HEX to Keil's CWD (output folder)
set "OUTPUT_HEX_PATH=%KEIL_CWD%\%MERGED_HEX_FILENAME%"

REM --- Echo Paths for Debugging ---
echo Project Root (derived from batch script location): "%PROJECT_ROOT%"
echo Keil Current Working Directory (at script execution): "%KEIL_CWD%"
echo ---
echo Bootloader HEX:  "%BOOTLOADER_HEX_PATH%"
echo Application HEX: "%APP_HEX_PATH%"
echo Output HEX:      "%OUTPUT_HEX_PATH%"
echo Python Script:   "%PYTHON_SCRIPT_PATH%"
echo ---

REM --- Check for Python ---
python --version >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo ERROR: Python not found or not in PATH.
    echo Please install Python 3.x and ensure it is added to your system PATH.
    goto EndScript
)

REM --- (Optional) Install Dependencies ---
REM Make sure requirements.txt is in the project root directory.
REM echo Checking Python dependencies...
REM python -m pip install -r "%PROJECT_ROOT%\requirements.txt%"
REM if %ERRORLEVEL% neq 0 (
REM     echo ERROR: Failed to install Python dependencies.
REM     goto EndScript
REM )

REM --- Run Merge Script ---
echo Running HEX merge script...
REM Change CWD to Project Root before running python script, if python script expects that.
REM Or, ensure python script handles paths correctly regardless of its CWD.
REM For now, assuming python script can handle absolute paths passed to it.

python "%PYTHON_SCRIPT_PATH%" "%BOOTLOADER_HEX_PATH%" "%APP_HEX_PATH%" "%OUTPUT_HEX_PATH%"

if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to merge HEX files. See output from python script above.
    goto EndScript
)

echo Successfully merged HEX files.
echo Output: "%OUTPUT_HEX_PATH%"

:EndScript
endlocal