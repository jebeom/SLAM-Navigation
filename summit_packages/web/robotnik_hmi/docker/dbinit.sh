#!/bin/bash

# Database initialization script
# This script is executed when the database container is created

echo 'Checking if db service is alive'
for i in $(seq 1 1000); do
	mysql \
		-h${DATABASE_HOSTNAME} \
		-uroot \
		-p${DB_ROOT_PASSWORD} \
		-e 'SELECT 1'
if ! [ $? -eq 0 ]; then
  echo 'connection failed'
  sleep 0.5
else
  echo 'The db service is alive'
  break
fi
done
echo 'Check if db is already installed'
mysql \
	-h${DATABASE_HOSTNAME} \
	-u${DB_USERNAME} \
	-p${DB_PASS} \
	-D${DB_DATABASE} \
	-e 'SELECT 1' \
;
if [ $? -eq 0 ]; then
	echo 'db already installed'
	exit 0
fi
set -x
echo 'Installing schema'
mysql \
	-h${DATABASE_HOSTNAME} \
	-uroot \
	-p${DB_ROOT_PASSWORD} \
	< /sql-data/db_robotnik.sql
if ! [ $? -eq 0 ]; then
	echo 'error installing sql data'
	exit 1
fi
	echo 'basic robotnik sql schema installed'
set -x
mysql \
	-h${DATABASE_HOSTNAME} \
	-uroot \
	-p${DB_ROOT_PASSWORD} \
-e "GRANT ALL ON *.* to ${DB_USERNAME}@'%' IDENTIFIED BY '${DB_PASS}';"
if ! [ $? -eq 0 ]; then
	echo 'error updating db password'
	exit 1
fi
echo 'Database access user password updated'
mysql \
	-h${DATABASE_HOSTNAME} \
	-u${DB_USERNAME} \
	-p${DB_PASS} \
	-D${DB_DATABASE} \
	-e \
	"UPDATE users SET usr_username=\"${WEB_MASTER_USR}\", usr_name=\"${WEB_MASTER_NAME}\", usr_password=\"${WEB_MASTER_PASS}\" WHERE   usr_id=1;"
if ! [ $? -eq 0 ]; then
  echo 'error updating web root credentials'
  exit 1
fi
echo 'updated web root credentialdds'
exit 0
