@echo off

for /r %%i in (*.wrl) do "C:\Program Files\VCG\MeshLab\meshlabserver" -i %%i -o %%~pi%%~ni.dae