#!/bin/sh

# script intended to release a new version into the release repository
# arguments: version
#            - version has to be an existing branch/tag of the repository

if [ "$#" -ne 2 ]; then
    echo "Use of the program: $0 version(existing tag) release_repository"
    exit 1
fi

ROOT_PATH=$(pwd)
TEMPORAL_DIRECTORY="./tmp"
RELEASE_REPOSITORY=$2
MASTER_BRANCH="master"
VERSION=$1
ROOT_PACKAGE_NAME=$(basename `git rev-parse --show-toplevel`)
# name of the folder to create the new release workspace
NEW_PACKAGE_FOLDER=$ROOT_PACKAGE_NAME\_release_$VERSION

echo "Releasing version $VERSION in $RELEASE_REPOSITORY..."
sleep 2

echo "Checking out to branch $VERSION"
git checkout $VERSION

if [ "$?" != "0" ]; then
    echo "The version $VERSION does not exist"
    exit 1
fi

if [ -d "$TEMPORAL_DIRECTORY" ]; then
    echo "Removing temporal directory $TEMPORAL_DIRECTORY"
    rm -rf $TEMPORAL_DIRECTORY
fi

if [ "$?" != "0" ]; then
    echo "Error removing $TEMPORAL_DIRECTORY"
    exit 1
fi

if [ -d "$NEW_PACKAGE_FOLDER" ]; then
    echo "Removing release directory $NEW_PACKAGE_FOLDER"
    rm -rf $NEW_PACKAGE_FOLDER
fi

if [ "$?" != "0" ]; then
    echo "Error removing $NEW_PACKAGE_FOLDER"
    exit 1
fi

if [ -f "$NEW_PACKAGE_FOLDER.tar.gz" ]; then
    echo "Removing release compressed file $NEW_PACKAGE_FOLDER.tar.gz"
    rm -rf $NEW_PACKAGE_FOLDER.tar.gz
fi

if [ "$?" != "0" ]; then
    echo "Error removing $NEW_PACKAGE_FOLDER.tar.gz"
    exit 1
fi

# Create a release workspace based on current branch
echo "Creating a release from branch $VERSION ..."
./build_system/create_package_with_debs.sh $NEW_PACKAGE_FOLDER

if [ "$?" != "0" ]; then
    echo "Error creating the release"
    exit 1
fi

echo "Removing all the .git info from all the submodules $VERSION ..."
cd $NEW_PACKAGE_FOLDER
find . \( -name ".git" -o -name ".gitignore" -o -name ".gitmodules" -o -name ".gitattributes" \) -exec rm -rf -- {} +
cd ..

echo "Creating temporal directory $TEMPORAL_DIRECTORY"
mkdir $TEMPORAL_DIRECTORY

if [ "$?" != "0" ]; then
    echo "Error creating $TEMPORAL_DIRECTORY"
    exit 1
fi

cd $TEMPORAL_DIRECTORY

git clone --single-branch --branch $MASTER_BRANCH $RELEASE_REPOSITORY

if [ "$?" != "0" ]; then
    echo "Error cloning $RELEASE_REPOSITORY"
    exit 1
fi

cd $(basename -s .git $RELEASE_REPOSITORY)

if [ "$?" != "0" ]; then
    echo "Error moving to folder $(basename -s .git $RELEASE_REPOSITORY)"
    exit 1
fi

git checkout --orphan temporal_branch

if [ "$?" != "0" ]; then
    echo "Error creating new branch $VERSION for release in repository $RELEASE_REPOSITORY"
    exit 1
fi

# clean branch
git rm -rf .

if [ "$?" != "0" ]; then
    echo "Error cleaning branch"
    exit 1
fi

# Brings release folder files into branch
mv -v $ROOT_PATH/$NEW_PACKAGE_FOLDER/* .
rm -r $ROOT_PATH/$NEW_PACKAGE_FOLDER

if [ "$?" != "0" ]; then
    echo "Error moving release folder $ROOT_PATH/$NEW_PACKAGE_FOLDER to current branch"
    exit 1
fi

# Add and commit
git add --all
git commit -m "New version $VERSION"
git tag $VERSION
git push origin $VERSION
if [ "$?" != "0" ]; then
    echo "Error pushing the tag $VERSION"
    exit 1
fi

# Removes temporal data
cd $ROOT_PATH
rm $NEW_PACKAGE_FOLDER.tar.gz
rm -rf $TEMPORAL_DIRECTORY

echo "New version $VERSION released!"
