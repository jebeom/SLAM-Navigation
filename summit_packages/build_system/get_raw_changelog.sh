#!/bin/bash

# Target tag (new tag)
target_tag="${1}"
if [ -z "${target_tag}" ]; then
    echo "No target tag provided"
    exit 1
fi

# Get the distro and version from the tag
distro=$(echo "${target_tag}" | cut -d'-' -f1)
version=$(echo "${target_tag}" | cut -d'-' -f2)

# Search for the previous tag (latest tag)
latest_tag=$(git describe --tags --abbrev=0 2>/dev/null)

# If no latest tag is found, get the first commit
if [ -z "${latest_tag}" ]; then
    latest_tag=$(git rev-list --max-parents=0 HEAD)
fi

echo -e "\e[1;34m[INFO]:\e[0m Target tag: ${target_tag}"
echo -e "\e[1;34m[INFO]:\e[0m Latest tag: ${latest_tag}"
# Wait for user input
read -p "Press enter to continue"

# Get the remote url
git_remote_url=$(git config --get remote.origin.url)

# If no remote url is found fails
if [ -z "${git_remote_url}" ]; then
    echo "No remote url found"
    exit 1
fi

git_remote_url_formatted=${git_remote_url%".git"}

# Get the changelog
echo "# ${distro}-${version} ($(date +%d-%m-%Y))" > .changelog
git log "${latest_tag}..HEAD" --pretty=format:"- %s (%an)([%h](${git_remote_url_formatted}/commit/%h))" --abbrev-commit >> .changelog
echo -e "\n" >> .changelog

# Emplace top changelog
cat .changelog CHANGELOG.md > CHANGELOG.md.tmp
rm .changelog
mv CHANGELOG.md.tmp CHANGELOG.md

# Commit and push
git add CHANGELOG.md
git commit -m "Bump to version ${distro}-${version}"
git tag "${distro}-${version}"
