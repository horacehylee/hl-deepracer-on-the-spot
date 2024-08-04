PURPOSE: This readme is to allow you to have AWS Service access via the command line for your sandbox.
         As part of this it provides you with temporary access credentials.
         This access is gained via the Sandbox Toolkit (pcl) and below are the instructions to get this.

PRE-REQUISITES:
- You have an RSA hard or soft token:
- You are working off-network on your own personal machine
- You have access to a command line tool - e.g. Git Bash


STEP 1: Download the latest pcl Toolkit (via the AWS Console)
- Login to the AWS Console via: https://idag2.jpmorganchase.com/adfs/ls/idpinitiatedsignon.aspx?logintorp=ASB
- (Ensure that you are working off-prem)
- (Ensure that you use your desktop password to login)
- In the AWS Console browser, open a new tab
- In this tab, open link: https://s3.console.aws.amazon.com/s3/buckets/l1-toolkit/?region=us-east-1&tab=overview
- Download the appropriate toolkit zip file
--- For Windows, pcl-..-windows-..-...zip
--- For linux, pcl-..-linux-..-...zip
--- For Mac, pcl-..-darwin-..-...zip
- Unzip this file

STEP 2: Run the pcl Toolkit
- Launch your command line tool (e.g. git bash)
- cd to the folder where you downloaded the pcl Toolkit
- ./pcl aws --sandbox-user --domain <your-domain> --sid <your-sid>  (optional) --profile-name <new_profile>

i.e.
- ./pcl aws --sandbox-user --domain ASIAPAC --sid O698267

- INFO: Default profile name is 'adfs' - or you can be specific by appending '--profile-name <new-profile-name>''
- Enter your desktop password
- Enter your RSA token id
- (Only for a new profile name AND you have access to multiple accounts) Select the account you wish to access

- INFO: Successful access will generate temporary credentials into your .aws credentials file with a profile-named entry
- INFO: These temporary credentials expire after 8 hours - simply run again to regenerate new temp credentials
- INFO To set a particular profile name to be your default then set the environment variable: AWS_DEFAULT_PROFILE

STEP 3: Run your aws cli commands/Use the temporary access key credentials
- Example: Enter your aws cli commands - using --profile adfs (or set AWS_DEFAULT_PROFILE)
- Example: aws s3 ls --profile adfs  (if AWS_DEFAULT_PROFILE is not set)

STEP 4: For more help/options
- ./pcl aws --help
- ./pcl --help

INFO: If you are using an IDE, you will need to set your session to use the AWS_DEFAULT_PROFILE also
to ensure that you pick up the correct session token when running the code

