VERSION=`git rev-parse --short=8 HEAD`
echo $VERSION
echo "#define GIT_HASH \"$VERSION\"" > ../Core/Inc/gitversion.h
touch ../Core/Src/main.c
