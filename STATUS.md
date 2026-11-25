# Build status

- result: **success**
- branch: main
- commit: d27a84900f5ab32016931aa14ee488a4f5ecefab
- message: Changelog: note that the repo is public for connector access
- compiler warnings: 0
- run: https://github.com/cloudmininggood-eng/flipper-trail/actions/runs/45543810730
- time: 2026-11-25 04:00:53 UTC

## Build log (last 150 lines)
```
04:00:38.070 [I] Deploying SDK for f7
04:00:38.071 [I] Fetching version info for UpdateChannel.RELEASE from https://up.momentum-fw.dev/firmware/directory.json
04:00:38.852 [I] Using version: mntm-012
04:00:38.852 [I] uFBT SDK dir: /home/runner/.ufbt/current
04:00:39.030 [I] Deploying SDK
04:00:39.282 [I] SDK deployed.
04:00:39.375 [I] uFBT version    0.2.6
04:00:39.376 [I] State dir       /home/runner/.ufbt
04:00:39.376 [I] Download dir    /home/runner/.ufbt/download
04:00:39.376 [I] SDK dir         /home/runner/.ufbt/current
04:00:39.376 [I] Toolchain dir   /home/runner/.ufbt/toolchain
04:00:39.376 [I] Target          f7
04:00:39.376 [I] Mode            channel
04:00:39.376 [I] Version         mntm-012
04:00:39.376 [I] Details         {'hw_target': 'f7', 'mode': 'channel', 'channel': 'release', 'json_index': 'https://up.momentum-fw.dev/firmware/directory.json', 'version': 'mntm-012'}
SDK API version: 87.1 (expected 87.1)
Checking for tar..yes
Checking if downloaded toolchain tgz exists..no
Checking curl..yes
Downloading toolchain:
                                                                           0.4%######                                                                     8.5%############                                                              17.4%###################                                                       26.9%#########################                                                 35.7%#############################                                             41.3%#################################                                         47.0%########################################                                  56.2%#############################################                             62.9%#################################################                         69.0%######################################################                    75.6%##########################################################                80.9%#############################################################             85.4%##################################################################        91.8%######################################################################    98.1%######################################################################## 100.0%
done
Removing old toolchain..done
Unpacking toolchain to '/home/runner/.ufbt/toolchain':
################################################################################ 100.0%
linking toolchain to 'current'..done
Cleaning up..done
scons: Entering directory `/home/runner/.ufbt/current/scripts/ufbt'
	CC	/home/runner/work/flipper-trail/flipper-trail/flipper_trail.c
	CDB	/home/runner/work/flipper-trail/flipper-trail/.vscode/compile_commands.json
	LINK	/home/runner/.ufbt/build/flipper_trail_d.elf
	APPMETA	/home/runner/.ufbt/build/flipper_trail.fap
	INSTALL	/home/runner/work/flipper-trail/flipper-trail/dist/debug/flipper_trail_d.elf
	FAP	/home/runner/.ufbt/build/flipper_trail.fap
	FASTFAP	/home/runner/.ufbt/build/flipper_trail.fap
	INSTALL	/home/runner/work/flipper-trail/flipper-trail/dist/flipper_trail.fap
	APPCHK	/home/runner/.ufbt/build/flipper_trail.fap
		Target: 7, API: 87.1
```
