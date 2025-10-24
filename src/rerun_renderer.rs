use rerun::{RecordingStream, RecordingStreamBuilder};
use robot_behavior::{AddRobot, Renderer, RendererResult, RobotFile};

use crate::{RerunRobot, rerun_robot::RerunRobotBuilder};

pub struct RerunHost {
    rec: RecordingStream,
    search_paths: Vec<std::path::PathBuf>,
}

impl RerunHost {
    pub fn new(app_name: &str) -> anyhow::Result<Self> {
        let rec = RecordingStreamBuilder::new(app_name).spawn()?;
        Ok(Self {
            rec,
            search_paths: vec![],
        })
    }
}

impl Renderer for RerunHost {
    fn set_additional_search_path(
        &mut self,
        path: impl AsRef<std::path::Path>,
    ) -> RendererResult<&mut Self> {
        self.search_paths.push(path.as_ref().to_path_buf());
        Ok(self)
    }
}

impl AddRobot for RerunHost {
    type PR<R> = RerunRobot<R>;
    type RB<'a, R: RobotFile> = RerunRobotBuilder<'a, R>;

    fn robot_builder<'a, R: RobotFile>(
        &'a mut self,
        name: impl ToString,
    ) -> RerunRobotBuilder<'a, R> {
        // 在 search_paths 中查找 URDF 文件所在目录
        let search_path = self.search_paths.iter().find_map(|path| {
            let urdf_path = path.join(R::URDF);
            if urdf_path.exists() {
                Some(path.clone())
            } else {
                None
            }
        });

        RerunRobotBuilder {
            _marker: std::marker::PhantomData,
            _rerun: &mut self.rec,
            name: name.to_string(),
            load_file: search_path.clone().unwrap_or_default().join(R::URDF),
            mesh_path: search_path,
            base: None,
            base_fixed: false,
            scaling: None,
        }
    }
}
