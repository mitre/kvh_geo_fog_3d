////////////////////////////////////////////////////////////////////////////////
//                               NOTES
// This build works assuming the following:
//  * Your code is a catkin package
//  * Your package.xml is correct, and thus will install your required
//    dependencies using rosdep.
//  * Your package passes the catkin linter (will thrown a warning if it fails)
////////////////////////////////////////////////////////////////////////////////

pipeline
{
    options
    {
        // Keep N most recent builds
        buildDiscarder(logRotator(numToKeepStr:'20'))
        timeout(time: 2, unit: 'HOURS') 
    }
    agent
    {
        label 'docker-ubuntu-1604'
    }
    triggers
    {
        //Trigger builds every night at 2AM
        cron('H 2 * * *')
    }
    stages
    {
        stage('Checkout')
        {
            steps
            {
                CheckoutKineticDevel()
            }
        } //end: stage('Checkout')
        stage('Setup')
        {
            steps
            {
                //Add useful SSH keys to known_hosts
                sh script: """
                    mkdir -p ~/.ssh
                    chmod 700 ~/.ssh
                    ssh-keyscan -H github.com >> ~/.ssh/known_hosts
                    ssh-keyscan -H gitlab.mitre.org >> ~/.ssh/known_hosts
                """, label: 'Setup github/gitlab SSH keys'
                sh script: """#!/bin/bash
                    sudo -E apt-get update --fix-missing
                """, label: 'Apt Cache Update'
                //Setup the OS, specifically for ROS Kinetic
                SetupKinetic()
            }
        } //end: stage('Setup')
        stage('Build')
        {
            steps
            {
                BuildRelease()
            }
        } //end: stage('Build')
        stage('Post-Build Code Analysis')
        {
            steps
            {
                script
                {
                    warnError('Statick Failed!')
                    {
                        RunStatickTools()
                    }
                }
            }
        } //end: stage('Code Analysis')
        stage('Test')
        {
            steps
            {
                BuildTest()
            }
        } //end: stage('Test')
        stage('Package')
        {
            steps
            {
                PackageDebian()
                //sh script: 'echo "Packaging disabled for now"', label: "Packaging"
            }
        }
    } //end: stages

    post
    {
        always
        {
	        sh 'tar -cjvf statick_results_mitre_ros_${BUILD_NUMBER}.tar.bz statick_output/all_packages-mitre_ros/*.json.statick || true'
            archiveArtifacts '*.tar.bz'
            archiveArtifacts 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml'

            ////////////////////////////////////////////////////////////////////
            // Due to how fragile plugin publishers are with Declarative
            // Pipelines in Jenkins right now, I'm wrapping all of the publisher
            // calls in warnError(). Once the plugin maintainers get their stuff
            // together, I will be smarter about this.
            ////////////////////////////////////////////////////////////////////
            script
            {
                //Statick
                warnError('Publishing Statick Results Failed!')
                {
                    //Using the warnings-ng plugin
		            recordIssues(
				        enabledForFailure: true,
				        qualityGates: [[threshold: 1, type: 'TOTAL_ERROR'],
                                       [threshold: 1, type: 'TOTAL_HIGH'],
                                       [threshold: 1, type: 'TOTAL_NORMAL', unstable: true]],
				        tools: [issues(name: 'Statick', pattern: 'statick_output/all_packages-*/*.json.statick')]
				)
                }
		//GCC warnings/errors recorded separately
		warnError('Publishing GCC Warnings/Errors Failed!')
		{
		    recordIssues(enabledForFailure: false, aggregatingResults : false, tool: gcc4())
		}
                //Unit Testing
                warnError('Publishing Unit Test Results Failed!')
                {
                    xunit (thresholds: [ skipped(failureThreshold: '0'), failed(failureThreshold: '0') ],
                        tools: [ GoogleTest(pattern: 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml') ])
                }
            }
        }
        failure
        {
            SendEmail()
        }
        fixed
        {
            SendEmail()
        }
    }
} //end: pipeline


void CheckoutKineticDevel()
{
    //Clone the repo
    checkout([$class: 'GitSCM',
        branches: [[name: '*/kinetic-devel']],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'RelativeTargetDirectory',
            relativeTargetDir: 'catkin_ws/src/kvh_geo_fog_3d']],
        submoduleCfg: [],
        userRemoteConfigs:
            [[credentialsId:'ZLAC_GITLAB_KEY',
            url: 'git@gitlab.mitre.org:DART/kvh_geo_fog_3d.git']]
        ])
}
void SetupKinetic()
{
    //Use rosdep to install dependencies for our package, as defined by its
    //package.xml. Assumes that the current user has nopasswd set for sudo,
    //since rosdep calls sudo for apt-get install.
    sh script: """
        cd catkin_ws
        rosdep update
        # Install all dependencies for packages, but ignore those which resolve to local packages. Automatically answer yes to questions.
        rosdep install --rosdistro kinetic --from-paths src --ignore-src -y -r src/kvh_geo_fog_3d
    """, label: 'Install Package rosdeps'
}
void BuildRelease()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/kinetic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCATKIN_ENABLE_TESTING=1''', label: 'Build Release'
}
void BuildTest()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/kinetic/setup.bash
        catkin build --verbose --catkin-make-args run_tests''', label: 'Build Test'
}
void BuildDebug()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        set -o pipefail
        source /opt/ros/kinetic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCATKIN_ENABLE_TESTING=1''', label: 'Build Debug'
}

void RunStatickTools()
{
    //Get our Statick config repo
    checkout([$class: 'GitSCM',
        branches: [[name: '*/master']],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'RelativeTargetDirectory',
            relativeTargetDir: 'statick-mitre-ros-configuration']],
        submoduleCfg: [],
        userRemoteConfigs:
            [[credentialsId:'ZLAC_GITLAB_KEY',
            url: 'git@gitlab.mitre.org:DART-release/statick-mitre-ros-configuration.git']]
        ])
    sh script: '''#!/bin/bash
        mkdir -p statick_output
	    echo "Starting statick runs"
	    mkdir -p statick_output
	    statick_ws catkin_ws/src/kvh_geo_fog_3d --output-directory statick_output --user-paths catkin_ws/src/kvh_geo_fog_3d/devops/statick_config,statick-mitre-ros-configuration/statick_config --profile mitre_ros.yaml
    ''', label: 'Statick Analysis Toolkit'
}

void PackageDebian()
{
    sh script: '''#!/bin/bash
        echo "Currently building debs is not supported! Can't handle missing deps"
    ''', label: "Debian Packaging"
//    sh script: '''#!/bin/bash
//        cd catkin_ws/src/kvh_geo_fog_3d
//        ./build_all_debs.sh
//    ''', label: "Debian Packaging"
}

void SendEmail()
{
    emailext body: "${currentBuild.currentResult}: Job ${env.JOB_NAME} build ${env.BUILD_NUMBER}\n More info at: ${env.BUILD_URL}",
                recipientProviders: [[$class: 'DevelopersRecipientProvider'], [$class: 'RequesterRecipientProvider']],
                subject: "Jenkins Build ${currentBuild.currentResult}: Job ${env.JOB_NAME}", to: 'zlacelle, tbostic'
}
