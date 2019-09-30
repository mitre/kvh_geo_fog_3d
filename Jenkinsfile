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
                CheckoutMaster()
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
                //Setup the OS, specifically for ROS Kinetic
                SetupKinetic()
            }
        } //end: stage('Setup')
        stage('Pre-Build Code Analysis')
        {
            steps
            {
                script
                {
                    warnError('Catkin Linter Failed!')
                    {
                        CatkinLint()
                    }
                    warnError('CppCheck Failed!')
                    {
                        CppCheck()
                    }
                    warnError('Lizard Complexity Analysis Failed!')
                    {
                        Lizard()
                    }
                }
            }
        } //end: stage('Code Analysis')
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
                    warnError('ClangTidy Failed!')
                    {
                        ClangTidy()
                    }
                    warnError('RosLint Failed!')
                    {
                        RosLint()
                    }
                }
            }
        }
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
            archiveArtifacts 'catkin_ws/*_lint.txt'
            archiveArtifacts 'cppcheck-result.xml'
            archiveArtifacts 'lizard.xml'
            archiveArtifacts 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml'
            archiveArtifacts 'catkin_ws/src/kvh_geo_fog_3d/clang_format_kvh_geo_fog_3d.tar.gz'
            
            ////////////////////////////////////////////////////////////////////
            // Due to how fragile plugin publishers are with Declarative
            // Pipelines in Jenkins right now, I'm wrapping all of the publisher
            // calls in warnError(). Once the plugin maintainers get their stuff
            // together, I will be smarter about this.
            ////////////////////////////////////////////////////////////////////
            script
            {
                //CPPCHECK
                warnError('Publishing CppCheck Results Failed!')
                {
                    //Using the warnings-ng plugin
                    recordIssues enabledForFailure: false, aggregatingResults : false, tool: cppCheck(pattern: 'cppcheck-result.xml')
                }
                //LIZARD
                //warnError('Publishing Lizard Results Failed!')
                //{
                //    step([$class: 'hudson.plugins.cppncss.CppNCSSPublisher', reportFilenamePattern: 'lizard.xml', functionCcnViolationThreshold: 5, functionNcssViolationThreshold: 10, targets: []])
                //}
                //GCC Warnings/Errors
                warnError('Publishing GCC Warnings/Errors Failed!')
                {
                    recordIssues enabledForFailure: false, aggregatingResults : false, tool: gcc4()
                }
                //Unit Testing
                warnError('Publishing Unit Test Results Failed!')
                {
                    xunit (thresholds: [ skipped(failureThreshold: '0'), failed(failureThreshold: '0') ],
                        tools: [ GoogleTest(pattern: 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml') ])
                }
                //Clang-tidy
                warnError('Publishing Clang-Tidy Results Failed!')
                {
                    //Use the old junit publisher
                    junit 'catkin_ws/src/kvh_geo_fog_3d/clangtidy/*_clangtidy.xml'
                    //xUnit publisher doesn't work well for JUnit-formatted files
                    //xunit([JUnit(deleteOutputFiles: true, failIfNotNew: false, pattern: 'catkin_ws/src/kvh_geo_fog_3d/clangtidy/*_clangtidy.xml', skipNoTestFiles: true, stopProcessingIfError: true)])
                }
            }
	}
	success
	{
            archiveArtifacts 'catkin_ws/src/*.deb'
            archiveArtifacts artifacts: 'catkin_ws/src/kvh_geo_fog_3d/clangtidy/*_clangtidy.xml', onlyIfSuccessful: true
            archiveArtifacts artifacts: 'catkin_ws/src/kvh_geo_fog_3d/roslint_output/*.txt', onlyIfSuccessful: true
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


void CheckoutMaster()
{
    //Clone the repo
    checkout([$class: 'GitSCM',
        branches: [[name: '*/master']],
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
        rosdep install --rosdistro kinetic --from-paths src -y -r src/kvh_geo_fog_3d
    """, label: 'Install Package rosdeps'
}
void BuildRelease()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/kinetic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=On''', label: 'Build Release'
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
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=On''', label: 'Build Debug'
}
void CatkinLint()
{
    sh script: '''#!/bin/bash
        cd catkin_ws
        set -o pipefail
        source /opt/ros/kinetic/setup.bash
        catkin lint -W2 src/kvh_geo_fog_3d |& tee catkinpackage_lint.txt
    ''', label: 'Catkin Linter'
}
void CppCheck()
{
    //Run cppcheck
    sh script: """
        # Run this from the base of the workspace! File paths are relative to
        # the run location, and Jenkins reports want those paths to be relative
        # to the workspace root (e.g. /home/jenkins/workspace/kvh_geo_fog_3d_driver)
        cppcheck --enable=warning,style,performance,portability --language=c++ --platform=unix64 --std=c++11 -I catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_driver/include/ --xml --xml-version=2 catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_driver/src catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_rviz/src 2> cppcheck-result.xml
    """, label: 'CPPCheck'
}
void Lizard()
{
    sh script: """
        lizard -l cpp catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_driver/src/ catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_driver/include/ catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_rviz/src/ catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_rviz/include/ --xml > lizard.xml 2>&1
    """, label: 'Lizard'
}
void ClangTidy()
{
    sh script: """#!/bin/bash
        cd catkin_ws/src/kvh_geo_fog_3d
        source ../../devel/setup.bash
        ./devops/clang_tidy.sh
    """, label: 'Clang-tidy'
}
void RosLint()
{
    sh script: """#!/bin/bash
        cd catkin_ws/src/kvh_geo_fog_3d
        source ../../devel/setup.bash
        ./devops/roslint.sh
    """, label: 'Roslint'
}
void PackageDebian()
{
    sh script: '''#!/bin/bash
        cd catkin_ws/src/kvh_geo_fog_3d
        ./build_all_debs.sh
    ''', label: "Debian Packaging"
}

void SendEmail()
{
    emailext body: "${currentBuild.currentResult}: Job ${env.JOB_NAME} build ${env.BUILD_NUMBER}\n More info at: ${env.BUILD_URL}",
                recipientProviders: [[$class: 'DevelopersRecipientProvider'], [$class: 'RequesterRecipientProvider']],
                subject: "Jenkins Build ${currentBuild.currentResult}: Job ${env.JOB_NAME}", to: 'zlacelle, tbostic'
}