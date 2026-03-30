@echo off
REM Javadoc Generation Script for FTC Robot Code
REM
REM This script generates JavaDoc HTML documentation for all robot code.
REM The documentation will be created in the javadoc/ directory.

echo Generating JavaDoc documentation...

javadoc ^
  -d "javadoc" ^
  -sourcepath "TeamCode/src/main/java" ^
  -subpackages org.firstinspires.ftc.teamcode ^
  -exclude "org.firstinspires.ftc.teamcode.samples" ^
  -private ^
  -author ^
  -version ^
  -use ^
  -link https://ftctechnh.github.io/ftc_app/doc/javadoc/ ^
  -doctitle "ARES FTC Robot Code Documentation" ^
  -windowtitle "ARES FTC Robot Code" ^
  -header "ARES FTC Robot Code" ^
  -footer "Generated for ARES FTC Team" ^
  -bottom "Copyright © 2026 ARES FTC. All rights reserved." ^
  -overview TeamCode/overview.html

echo.
echo JavaDoc generation complete!
echo Documentation created in: javadoc/index.html
echo.
pause
