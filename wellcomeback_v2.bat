@echo off
ren *.tempname *.
for /f %%i in ('"dir /ad/s/b"') do (
cd %%i 
attrib -s -h -r
ren *.tempname *.
echo %%i
)
pause